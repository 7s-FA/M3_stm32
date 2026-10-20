#include "drive_control.h"
#include "app_config.h"
#include "xl430.h"
#include "battery_monitor.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <rmw_microros/rmw_microros.h>

/* All state and callbacks are owned by the single application task. */

typedef struct
{
    XL430_Handle_t motor;
    int32_t velocity_command;
    uint8_t velocity_command_pending;
    bool velocity_ready;
    bool drive_enabled;
    int32_t active_velocity;
    uint32_t last_velocity_command_tick;
    int32_t state;
    int32_t present_velocity;
    int32_t present_position;
    int32_t position_zero;
    bool position_zero_valid;
} WheelControl_t;

typedef enum
{
    WHEEL_COMMAND_SOURCE_CMD_VEL = 0,
    WHEEL_COMMAND_SOURCE_RAW_TEST
} WheelCommandSource_t;

static bool wheel_sample_valid;
static bool wheel_sample_received;
static uint32_t wheel_sample_tick;
static XL430_Result_t wheel_read_result;
static uint32_t wheel_read_retry_delay_ms;
static uint32_t last_wheel_read_error_tick;
static WheelControl_t left_wheel;
static WheelControl_t right_wheel;
static float cmd_linear_mps;
static float cmd_angular_radps;
static uint32_t last_cmd_vel_tick;
static uint8_t cmd_vel_received;
static WheelCommandSource_t wheel_command_source = WHEEL_COMMAND_SOURCE_CMD_VEL;

static void WheelInitialize(WheelControl_t *wheel, uint8_t id);
static void WheelSetTorque(WheelControl_t *wheel, bool enable);
static void WheelsProcessVelocityCommand(void);
static void WheelsUpdate(void);
static void JointStateUpdate(sensor_msgs__msg__JointState *message);
static int32_t WheelsCombinedState(const WheelControl_t *left,
                                   const WheelControl_t *right);
static void DriveProcessCmdVel(void);

/* Move state and its last result survive ROS reconnection, but not MCU reset.
 * IDs are monotonically increasing per boot; a duplicate last ID never moves
 * again. Rejected commands have a separate acknowledgement, preserving DONE. */
typedef enum { MOVE_IDLE, MOVE_STOPPING, MOVE_MOVING,
               MOVE_DONE, MOVE_ERROR, MOVE_CANCELED } MovePhase;
static struct {
    MovePhase phase;
    uint32_t id, request_id, started, settled_at;
    bool settling;
    int32_t left, right, speed, left_target, right_target;
    const char *ack;
    const char *reason;
} move;

static bool MoveActive(void)
{
    return move.phase == MOVE_STOPPING || move.phase == MOVE_MOVING;
}

static void ClearVelocityCommands(void)
{
    cmd_linear_mps = cmd_angular_radps = 0;
    cmd_vel_received = 0;
    wheel_command_source = WHEEL_COMMAND_SOURCE_CMD_VEL;
    left_wheel.velocity_command = right_wheel.velocity_command = 0;
    left_wheel.velocity_command_pending = right_wheel.velocity_command_pending = 0;
    left_wheel.active_velocity = right_wheel.active_velocity = 0;
}

static void MoveFail(const char *reason)
{
    /* Both shutdowns are attempted even if one UART transaction fails. */
    move.phase = MOVE_ERROR;
    move.reason = reason;
    ClearVelocityCommands();
    WheelSetTorque(&left_wheel, false);
    WheelSetTorque(&right_wheel, false);
}

size_t DriveControl_FormatMoveStatus(char *buffer, size_t capacity)
{
    static const char *const names[] = {
        "IDLE", "STOPPING", "MOVING", "DONE", "ERROR", "CANCELED"
    };
    int n = snprintf(buffer, capacity,
        "{\"id\":%lu,\"state\":\"%s\",\"reason\":\"%s\","
        "\"request_id\":%lu,\"ack\":\"%s\"}",
        (unsigned long)move.id, names[move.phase], move.reason ? move.reason : "none",
        (unsigned long)move.request_id, move.ack ? move.ack : "none");
    if (n < 0 || (size_t)n >= capacity) {
        if (capacity) buffer[0] = '\0';
        return 0;
    }
    return (size_t)n;
}

void DriveControl_MoveCommand(const char *command, size_t length)
{
    int64_t args[4] = {0};
    size_t at = 4, count = 0;
    move.request_id = 0;
    move.ack = "INVALID";
    if (!command || length >= XL430_CMD_BUFFER_SIZE || length < 6 ||
        memcmp(command, "move ", 5) != 0) return;
    while (at < length) {
        while (at < length && command[at] == ' ') ++at;
        if (at == length) break;
        if (count == 4) return;
        bool negative = command[at] == '-';
        if (negative || command[at] == '+') ++at;
        size_t digits = 0;
        int64_t value = 0;
        while (at < length && command[at] >= '0' && command[at] <= '9') {
            if (++digits > 10) return;
            value = value * 10 + command[at++] - '0';
        }
        if (!digits || (at < length && command[at] != ' ')) return;
        args[count++] = negative ? -value : value;
    }
    if (count != 3 && count != 4) return;
    if (count == 4) {
        if (args[3] <= 0 || args[3] > UINT32_MAX) return;
        move.request_id = (uint32_t)args[3];
    }
    if (args[0] < -MOVE_MAX_PULSES || args[0] > MOVE_MAX_PULSES ||
        args[1] < -MOVE_MAX_PULSES || args[1] > MOVE_MAX_PULSES ||
        args[2] < 1 || args[2] > XL430_SAFE_VELOCITY_LIMIT) return;
    if (count == 4 && move.request_id == move.id) {
        move.ack = (args[0] == move.left && args[1] == move.right &&
                    args[2] == move.speed) ? "DUPLICATE" : "ID_CONFLICT";
        return;
    }
    if (count == 4 && move.request_id < move.id) { move.ack = "STALE_ID"; return; }
    if (MoveActive()) { move.ack = "BUSY"; return; }
    if (!left_wheel.drive_enabled || !right_wheel.drive_enabled) {
        move.ack = "DISABLED"; return;
    }
    if (count == 3) {
        if (move.id == UINT32_MAX) { move.ack = "ID_EXHAUSTED"; return; }
        move.request_id = move.id + 1U;
    }
    move.id = move.request_id;
    move.left = (int32_t)args[0]; move.right = (int32_t)args[1];
    move.speed = (int32_t)args[2];
    move.phase = MOVE_STOPPING;
    move.ack = "ACCEPTED"; move.reason = "none";
    move.started = HAL_GetTick(); move.settling = false;
    ClearVelocityCommands();
}

static bool Near(int32_t a, int32_t b, int32_t tolerance)
{
    int64_t difference = (int64_t)a - b;
    return difference >= -tolerance && difference <= tolerance;
}

/* Rebase only at a mode transition. Keep published wheel angles continuous
 * when the actuator resets its multi-turn counter. Torque-off drift cannot be
 * distinguished from a reset here and must be checked on the actual chassis. */
static void RebaseWheel(WheelControl_t *wheel, const XL430_WheelState_t *state)
{
    if (wheel->position_zero_valid) {
        uint32_t zero = (uint32_t)wheel->position_zero +
                        (uint32_t)state->position - (uint32_t)wheel->present_position;
        memcpy(&wheel->position_zero, &zero, sizeof(zero));
    } else {
        wheel->position_zero = state->position;
        wheel->position_zero_valid = true;
    }
    wheel->present_position = state->position;
    wheel->present_velocity = state->velocity;
}

static bool ReadTransitionState(void)
{
    XL430_WheelState_t l, r;
    XL430_Result_t result = XL430_SyncReadWheelState(
        &left_wheel.motor, &l, &right_wheel.motor, &r);
    wheel_read_result = result;
    wheel_sample_valid = result == XL430_OK;
    if (result != XL430_OK) return false;
    RebaseWheel(&left_wheel, &l); RebaseWheel(&right_wheel, &r);
    wheel_sample_received = true; wheel_sample_tick = HAL_GetTick();
    wheel_read_retry_delay_ms = 0;
    return true;
}

static bool CheckTorque(void)
{
    bool l = false, r = false;
    return XL430_ReadTorque(&left_wheel.motor, &l) == XL430_OK && l &&
           XL430_ReadTorque(&right_wheel.motor, &r) == XL430_OK && r;
}

static void StartPositionMove(void)
{
    /* Both torque outputs off before configuring either actuator. */
    XL430_Result_t l = XL430_SetTorque(&left_wheel.motor, false);
    XL430_Result_t r = XL430_SetTorque(&right_wheel.motor, false);
    left_wheel.velocity_ready = right_wheel.velocity_ready = false;
    if (l != XL430_OK || r != XL430_OK) { MoveFail("torque_off"); return; }
    l = XL430_ConfigurePositionControl(&left_wheel.motor, move.speed,
        MOVE_PROFILE_ACCELERATION, XL430_BUS_WATCHDOG_TICKS);
    if (l == XL430_OK) r = XL430_ConfigurePositionControl(&right_wheel.motor,
        move.speed, MOVE_PROFILE_ACCELERATION, XL430_BUS_WATCHDOG_TICKS);
    if (l != XL430_OK || r != XL430_OK || !ReadTransitionState()) {
        MoveFail("position_setup"); return;
    }
    /* Never enable position torque with an old goal still in the register. */
    l = XL430_SetGoalPosition(&left_wheel.motor, left_wheel.present_position);
    r = XL430_SetGoalPosition(&right_wheel.motor, right_wheel.present_position);
    if (l != XL430_OK || r != XL430_OK) { MoveFail("hold_goal"); return; }
    l = XL430_SetTorque(&left_wheel.motor, true);
    if (l == XL430_OK) r = XL430_SetTorque(&right_wheel.motor, true);
    if (l != XL430_OK || r != XL430_OK || !CheckTorque() || !ReadTransitionState()) {
        MoveFail("position_enable"); return;
    }
    int64_t lt = (int64_t)left_wheel.present_position +
                 (LEFT_MOTOR_DIRECTION < 0 ? -move.left : move.left);
    int64_t rt = (int64_t)right_wheel.present_position +
                 (RIGHT_MOTOR_DIRECTION < 0 ? -move.right : move.right);
    if (lt < -1048575 || lt > 1048575 || rt < -1048575 || rt > 1048575) {
        MoveFail("goal_range"); return;
    }
    move.left_target = (int32_t)lt; move.right_target = (int32_t)rt;
    l = XL430_SyncWriteGoalPosition(&left_wheel.motor, move.left_target,
                                    &right_wheel.motor, move.right_target);
    if (l != XL430_OK) { MoveFail("goal_write"); return; }
    move.phase = MOVE_MOVING; move.started = HAL_GetTick(); move.settling = false;
}

static void FinishPositionMove(void)
{
    /* Both wheels are stationary. Do not publish DONE until mode restoration,
     * torque readback and a fresh paired sample have all succeeded. */
    XL430_Result_t l = XL430_SetTorque(&left_wheel.motor, false);
    XL430_Result_t r = XL430_SetTorque(&right_wheel.motor, false);
    if (l != XL430_OK || r != XL430_OK) { MoveFail("restore_torque"); return; }
    l = XL430_ConfigureVelocityControl(&left_wheel.motor, XL430_BUS_WATCHDOG_TICKS);
    if (l == XL430_OK)
        r = XL430_ConfigureVelocityControl(&right_wheel.motor, XL430_BUS_WATCHDOG_TICKS);
    if (l != XL430_OK || r != XL430_OK || !CheckTorque() || !ReadTransitionState()) {
        MoveFail("restore_mode"); return;
    }
    left_wheel.velocity_ready = right_wheel.velocity_ready = true;
    ClearVelocityCommands();
    move.phase = MOVE_DONE; move.reason = "none";
}

static void MoveUpdate(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - move.started) >=
        (move.phase == MOVE_STOPPING ? MOVE_STOP_TIMEOUT_MS : MOVE_TIMEOUT_MS)) {
        MoveFail("timeout"); return;
    }
    if (move.phase == MOVE_STOPPING) {
        /* Configure zero velocity first when coming from torque-on startup. */
        if (!left_wheel.velocity_ready || !right_wheel.velocity_ready) {
            XL430_Result_t l = XL430_ConfigureVelocityControl(
                &left_wheel.motor, XL430_BUS_WATCHDOG_TICKS);
            XL430_Result_t r = XL430_ConfigureVelocityControl(
                &right_wheel.motor, XL430_BUS_WATCHDOG_TICKS);
            if (l != XL430_OK || r != XL430_OK || !ReadTransitionState()) {
                MoveFail("stop_setup"); return;
            }
            left_wheel.velocity_ready = right_wheel.velocity_ready = true;
        }
        if (XL430_SyncWriteGoalVelocity(&left_wheel.motor, 0,
                                       &right_wheel.motor, 0) != XL430_OK) {
            MoveFail("stop_write"); return;
        }
    }
    WheelsUpdate();
    if (!wheel_sample_valid) { MoveFail("feedback"); return; }
    bool settled = Near(left_wheel.present_velocity, 0, MOVE_STOP_VELOCITY) &&
                   Near(right_wheel.present_velocity, 0, MOVE_STOP_VELOCITY);
    if (move.phase == MOVE_MOVING)
        settled = settled && Near(left_wheel.present_position, move.left_target,
                                  MOVE_POSITION_TOLERANCE) &&
                             Near(right_wheel.present_position, move.right_target,
                                  MOVE_POSITION_TOLERANCE);
    if (!settled) { move.settling = false; return; }
    if (!move.settling) { move.settling = true; move.settled_at = HAL_GetTick(); return; }
    if ((uint32_t)(HAL_GetTick() - move.settled_at) < MOVE_SETTLE_MS) return;
    if (!CheckTorque()) { MoveFail("torque_lost"); return; }
    if (move.phase == MOVE_STOPPING) StartPositionMove();
    else FinishPositionMove();
}

void DriveControl_Init(void)
{
    memset(&move, 0, sizeof(move));
    ClearVelocityCommands();
    wheel_sample_valid = false;
    wheel_sample_received = false;
    wheel_sample_tick = 0U;
    wheel_read_result = XL430_OK;
    wheel_read_retry_delay_ms = 0U;
    WheelInitialize(&left_wheel, XL430_LEFT_ID);
    WheelInitialize(&right_wheel, XL430_RIGHT_ID);
    BatteryMonitor_Init(&left_wheel.motor);
}

void DriveControl_SetTorque(bool enabled)
{
    if (MoveActive()) {
        if (enabled) return;
        move.phase = MOVE_CANCELED;
        move.reason = "motor_off";
    }
    if (!enabled) ClearVelocityCommands();
    WheelSetTorque(&left_wheel, enabled);
    WheelSetTorque(&right_wheel, enabled);
}

/* Called only from the application task when the Agent/session is lost. */
void DriveControl_Disconnect(void)
{
    cmd_linear_mps = cmd_angular_radps = 0.0f;
    cmd_vel_received = 0U;
    wheel_command_source = WHEEL_COMMAND_SOURCE_CMD_VEL;
    left_wheel.velocity_command = right_wheel.velocity_command = 0;
    left_wheel.velocity_command_pending = right_wheel.velocity_command_pending = 0U;
    DriveControl_SetTorque(false);
    if (move.phase == MOVE_CANCELED) move.reason = "disconnected";
}

void DriveControl_SetVelocity(float linear_mps, float angular_radps)
{
    if (MoveActive()) return;
    cmd_linear_mps = linear_mps;
    cmd_angular_radps = angular_radps;
    last_cmd_vel_tick = HAL_GetTick();
    cmd_vel_received = 1U;
    wheel_command_source = WHEEL_COMMAND_SOURCE_CMD_VEL;
}

void DriveControl_SetRawVelocity(bool left, int32_t velocity)
{
    if (MoveActive()) return;
    WheelControl_t *wheel = left ? &left_wheel : &right_wheel;
    wheel_command_source = WHEEL_COMMAND_SOURCE_RAW_TEST;
    wheel->velocity_command = velocity;
    wheel->velocity_command_pending = 1U;
}

void DriveControl_Update(void)
{
    if (MoveActive()) MoveUpdate();
    else {
        DriveProcessCmdVel();
        WheelsProcessVelocityCommand();
        WheelsUpdate();
    }
    BatteryMonitor_Update();
}

bool DriveControl_IsEnabled(void)
{
    return left_wheel.drive_enabled || right_wheel.drive_enabled;
}

void DriveControl_GetState(DriveState_t *state)
{
    state->left_state = left_wheel.state;
    state->right_state = right_wheel.state;
    state->combined_state = WheelsCombinedState(&left_wheel, &right_wheel);
    state->left_velocity = left_wheel.present_velocity;
    state->right_velocity = right_wheel.present_velocity;
    state->left_position = left_wheel.present_position;
    state->right_position = right_wheel.present_position;
    state->sample_valid = wheel_sample_valid;
    state->sample_tick_ms = wheel_sample_tick;
    state->sample_age_ms = wheel_sample_received ?
        (uint32_t)(HAL_GetTick() - wheel_sample_tick) : UINT32_MAX;
    state->read_result = (int32_t)wheel_read_result;
}

bool DriveControl_UpdateJointState(sensor_msgs__msg__JointState *message)
{
    if (!wheel_sample_valid || !rmw_uros_epoch_synchronized() ||
        !left_wheel.position_zero_valid || !right_wheel.position_zero_valid)
        return false;
    JointStateUpdate(message);
    return true;
}


static void WheelInitialize(WheelControl_t *wheel, uint8_t id)
{
    bool torque_enabled = false;
    XL430_Result_t result;

    memset(wheel, 0, sizeof(*wheel));
    result = XL430_Init(&wheel->motor, &huart1, id);
    if (result == XL430_OK)
    {
        result = XL430_Ping(&wheel->motor, NULL, NULL);
    }
    if (result == XL430_OK)
    {
        result = XL430_SetTorque(&wheel->motor, false);
    }
    if (result == XL430_OK)
    {
        result = XL430_ReadTorque(&wheel->motor, &torque_enabled);
    }

    wheel->state = (result == XL430_OK)
                 ? (torque_enabled ? 1 : 0)
                 : -(int32_t)result;
}

static void WheelSetTorque(WheelControl_t *wheel, bool enable)
{
    bool torque_enabled = false;
    XL430_Result_t result;


    if ((enable == false) && wheel->velocity_ready)
    {
        (void)XL430_SetGoalVelocity(&wheel->motor, 0);
        wheel->active_velocity = 0;
    }

    /* A canceled position move may leave mode 4 selected. Always establish
     * zero-speed mode before re-enabling, so no stale goal can run. */
    if (enable && !wheel->velocity_ready) {
        result = XL430_ConfigureVelocityControl(&wheel->motor, XL430_BUS_WATCHDOG_TICKS);
        wheel->velocity_ready = result == XL430_OK;
    } else result = XL430_SetTorque(&wheel->motor, enable);
    if (result == XL430_OK)
    {
        result = XL430_ReadTorque(&wheel->motor, &torque_enabled);
    }

    if (enable == false)
    {
        wheel->velocity_ready = false;
        wheel->drive_enabled = false;
        wheel->position_zero_valid = false;
    }
    else
    {
        wheel->drive_enabled = (result == XL430_OK) && torque_enabled;
    }

    wheel->state = (result == XL430_OK)
                 ? (torque_enabled ? 1 : 0)
                 : -(int32_t)result;
}

static void WheelsProcessVelocityCommand(void)
{
    int32_t left_velocity;
    int32_t right_velocity;
    uint32_t command_tick;
    XL430_Result_t left_result = XL430_OK;
    XL430_Result_t right_result = XL430_OK;
    XL430_Result_t sync_result;

    if ((left_wheel.velocity_command_pending == 0U) &&
        (right_wheel.velocity_command_pending == 0U))
    {
        return;
    }

    /* A raw test may update one wheel only; keep the other active command. */
    left_velocity = (left_wheel.velocity_command_pending != 0U)
                  ? left_wheel.velocity_command
                  : left_wheel.active_velocity;
    right_velocity = (right_wheel.velocity_command_pending != 0U)
                   ? right_wheel.velocity_command
                   : right_wheel.active_velocity;
    left_wheel.velocity_command_pending = 0U;
    right_wheel.velocity_command_pending = 0U;

    if (left_velocity > XL430_SAFE_VELOCITY_LIMIT)
    {
        left_velocity = XL430_SAFE_VELOCITY_LIMIT;
    }
    else if (left_velocity < -XL430_SAFE_VELOCITY_LIMIT)
    {
        left_velocity = -XL430_SAFE_VELOCITY_LIMIT;
    }

    if (right_velocity > XL430_SAFE_VELOCITY_LIMIT)
    {
        right_velocity = XL430_SAFE_VELOCITY_LIMIT;
    }
    else if (right_velocity < -XL430_SAFE_VELOCITY_LIMIT)
    {
        right_velocity = -XL430_SAFE_VELOCITY_LIMIT;
    }

    left_wheel.velocity_command = left_velocity;
    right_wheel.velocity_command = right_velocity;

    if ((left_wheel.drive_enabled == false) ||
        (right_wheel.drive_enabled == false))
    {
        left_wheel.active_velocity = 0;
        right_wheel.active_velocity = 0;
        /* Keep each wheel's torque/readback result, including UART errors.
         * A disabled peer blocks motion but must not hide the failure. */
        return;
    }

    /* Configuration writes require individual Status Packet responses. */
    if (left_wheel.velocity_ready == false)
    {

        left_result = XL430_ConfigureVelocityControl(
            &left_wheel.motor, XL430_BUS_WATCHDOG_TICKS);
        left_wheel.velocity_ready = (left_result == XL430_OK);
    }

    if (right_wheel.velocity_ready == false)
    {

        right_result = XL430_ConfigureVelocityControl(
            &right_wheel.motor, XL430_BUS_WATCHDOG_TICKS);
        right_wheel.velocity_ready = (right_result == XL430_OK);
    }

    if (left_result != XL430_OK)
    {
        left_wheel.state = -(int32_t)left_result;
    }
    if (right_result != XL430_OK)
    {
        right_wheel.state = -(int32_t)right_result;
    }
    if ((left_wheel.velocity_ready == false) ||
        (right_wheel.velocity_ready == false))
    {
        return;
    }

    sync_result = XL430_SyncWriteGoalVelocity(
        &left_wheel.motor, left_velocity,
        &right_wheel.motor, right_velocity);
    if (sync_result != XL430_OK)
    {
        left_wheel.state = -(int32_t)sync_result;
        right_wheel.state = -(int32_t)sync_result;
        return;
    }

    command_tick = HAL_GetTick();
    left_wheel.active_velocity = left_velocity;
    right_wheel.active_velocity = right_velocity;
    left_wheel.last_velocity_command_tick = command_tick;
    right_wheel.last_velocity_command_tick = command_tick;
    left_wheel.state = 1;
    right_wheel.state = 1;
}

static void WheelsUpdate(void)
{
    wheel_sample_valid = false;
    XL430_WheelState_t left_state;
    XL430_WheelState_t right_state;
    XL430_Result_t result;

    /* Raw test commands have a secondary timeout; stop both wheels together. */
    if (((left_wheel.active_velocity != 0) &&
         ((HAL_GetTick() - left_wheel.last_velocity_command_tick) >=
          XL430_COMMAND_TIMEOUT_MS)) ||
        ((right_wheel.active_velocity != 0) &&
         ((HAL_GetTick() - right_wheel.last_velocity_command_tick) >=
          XL430_COMMAND_TIMEOUT_MS)))
    {
        result = XL430_SyncWriteGoalVelocity(
            &left_wheel.motor, 0,
            &right_wheel.motor, 0);
        if (result != XL430_OK)
        {

            left_wheel.state = -(int32_t)result;
            right_wheel.state = -(int32_t)result;
            return;
        }

        left_wheel.active_velocity = 0;
        right_wheel.active_velocity = 0;
    }

    /* Feedback is independent of torque/velocity configuration. Keep the
     * command watchdog above this backoff so waiting never skips stop checks. */
    if (wheel_read_retry_delay_ms != 0U &&
        (uint32_t)(HAL_GetTick() - last_wheel_read_error_tick) < wheel_read_retry_delay_ms)
        return;
    result = XL430_SyncReadWheelState(
        &left_wheel.motor, &left_state,
        &right_wheel.motor, &right_state);
    wheel_read_result = result;
    if (result == XL430_OK)
    {
        wheel_read_retry_delay_ms = 0U;
        wheel_sample_valid = true;
        wheel_sample_received = true;
        wheel_sample_tick = HAL_GetTick();
        left_wheel.present_velocity = left_state.velocity;
        left_wheel.present_position = left_state.position;
        right_wheel.present_velocity = right_state.velocity;
        right_wheel.present_position = right_state.position;

        if (left_wheel.position_zero_valid == false)
        {
            left_wheel.position_zero = left_state.position;
            left_wheel.position_zero_valid = true;
        }
        if (right_wheel.position_zero_valid == false)
        {
            right_wheel.position_zero = right_state.position;
            right_wheel.position_zero_valid = true;
        }

        left_wheel.state = left_wheel.drive_enabled ? 1 : 0;
        right_wheel.state = right_wheel.drive_enabled ? 1 : 0;
    }
    else
    {
        /* Yield to the application loop after a transient failure. Repeated
         * failures back off without sleeping or blocking command handling. */
        if (wheel_read_retry_delay_ms == 0U)
            wheel_read_retry_delay_ms = WHEEL_READ_RETRY_FIRST_MS;
        else if (wheel_read_retry_delay_ms == WHEEL_READ_RETRY_FIRST_MS)
            wheel_read_retry_delay_ms = WHEEL_READ_RETRY_BACKOFF_MS;
        else if (wheel_read_retry_delay_ms < WHEEL_READ_RETRY_MAX_MS) {
            wheel_read_retry_delay_ms *= 2U;
            if (wheel_read_retry_delay_ms > WHEEL_READ_RETRY_MAX_MS)
                wheel_read_retry_delay_ms = WHEEL_READ_RETRY_MAX_MS;
        }
        last_wheel_read_error_tick = HAL_GetTick();
        left_wheel.state = -(int32_t)result;
        right_wheel.state = -(int32_t)result;
    }
}

static void JointStateUpdate(sensor_msgs__msg__JointState *message)
{
    uint32_t left_delta_raw;
    uint32_t right_delta_raw;
    int32_t left_delta;
    int32_t right_delta;
    int64_t epoch_nanos;

    if (message == NULL)
    {
        return;
    }

    /* Unsigned subtraction keeps the delta well-defined across int32 wrap. */
    left_delta_raw = (uint32_t)left_wheel.present_position
                   - (uint32_t)left_wheel.position_zero;
    right_delta_raw = (uint32_t)right_wheel.present_position
                    - (uint32_t)right_wheel.position_zero;
    memcpy(&left_delta, &left_delta_raw, sizeof(left_delta));
    memcpy(&right_delta, &right_delta_raw, sizeof(right_delta));

    message->position.data[0] =
        (double)left_delta
        * XL430_POSITION_RAD_PER_RAW
        * LEFT_JOINT_DIRECTION;
    message->position.data[1] =
        (double)right_delta
        * XL430_POSITION_RAD_PER_RAW
        * RIGHT_JOINT_DIRECTION;

    message->velocity.data[0] =
        (double)left_wheel.present_velocity
        * XL430_VELOCITY_RAD_S_PER_RAW
        * LEFT_JOINT_DIRECTION;
    message->velocity.data[1] =
        (double)right_wheel.present_velocity
        * XL430_VELOCITY_RAD_S_PER_RAW
        * RIGHT_JOINT_DIRECTION;

    epoch_nanos = rmw_uros_epoch_nanos();
    if (epoch_nanos > 0)
    {
        message->header.stamp.sec =
            (int32_t)(epoch_nanos / 1000000000LL);
        message->header.stamp.nanosec =
            (uint32_t)(epoch_nanos % 1000000000LL);
    }
    else
    {
        message->header.stamp.sec = 0;
        message->header.stamp.nanosec = 0U;
    }
}

static int32_t WheelsCombinedState(const WheelControl_t *left,
                                   const WheelControl_t *right)
{
    if (left->state < 0)
    {
        return left->state;
    }
    if (right->state < 0)
    {
        return right->state;
    }
    return ((left->state == 1) && (right->state == 1)) ? 1 : 0;
}

static void DriveProcessCmdVel(void)
{
    float linear_mps;
    float angular_radps;
    float left_mps;
    float right_mps;
    float raw_per_mps;
    float left_raw;
    float right_raw;
    float left_magnitude;
    float right_magnitude;
    float maximum_raw;
    float scale;

    /* Keep the legacy raw wheel topics available for motor bench tests. */
    if (wheel_command_source != WHEEL_COMMAND_SOURCE_CMD_VEL)
    {
        return;
    }

    /* Stop locally if cmd_vel has never arrived or has become stale. */
    if ((cmd_vel_received == 0U) ||
        ((HAL_GetTick() - last_cmd_vel_tick) >= CMD_VEL_TIMEOUT_MS))
    {
        linear_mps = 0.0f;
        angular_radps = 0.0f;
    }
    else
    {
        linear_mps = cmd_linear_mps;
        angular_radps = cmd_angular_radps;
    }

    left_mps = linear_mps
             - (angular_radps * WHEEL_SEPARATION_M * 0.5f);
    right_mps = linear_mps
              + (angular_radps * WHEEL_SEPARATION_M * 0.5f);

    /* Convert wheel linear velocity [m/s] to XL430 velocity raw units. */
    raw_per_mps = 60.0f
                / (2.0f * 3.14159265359f
                   * WHEEL_RADIUS_M
                   * XL430_VELOCITY_UNIT_RPM);

    left_raw = left_mps * raw_per_mps * LEFT_MOTOR_DIRECTION;
    right_raw = right_mps * raw_per_mps * RIGHT_MOTOR_DIRECTION;

    /* Scale both wheels together so the commanded curvature is preserved. */
    left_magnitude = (left_raw >= 0.0f) ? left_raw : -left_raw;
    right_magnitude = (right_raw >= 0.0f) ? right_raw : -right_raw;
    maximum_raw = (left_magnitude > right_magnitude)
                ? left_magnitude : right_magnitude;

    if (maximum_raw > (float)XL430_SAFE_VELOCITY_LIMIT)
    {
        scale = (float)XL430_SAFE_VELOCITY_LIMIT / maximum_raw;
        left_raw *= scale;
        right_raw *= scale;
    }

    left_wheel.velocity_command = (int32_t)left_raw;
    right_wheel.velocity_command = (int32_t)right_raw;
    left_wheel.velocity_command_pending = 1U;
    right_wheel.velocity_command_pending = 1U;
}
