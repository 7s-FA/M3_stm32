#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define XL430_CMD_BUFFER_SIZE        80
#define MOVE_MAX_PULSES              40960
#define MOVE_PROFILE_ACCELERATION   10U
#define MOVE_POSITION_TOLERANCE     2
#define MOVE_STOP_VELOCITY           1
#define MOVE_SETTLE_MS               200U
#define MOVE_STOP_TIMEOUT_MS         3000U
#define MOVE_TIMEOUT_MS              60000U
#define XL430_LEFT_ID                1U
#define XL430_RIGHT_ID               2U
#define XL430_SAFE_VELOCITY_LIMIT    100
#define XL430_COMMAND_TIMEOUT_MS     1000U
#define XL430_BUS_WATCHDOG_TICKS     50U
#define DRIVE_CONTROL_PERIOD_MS      20U
#define DRIVE_STATE_PUBLISH_MS       100U
/* Minimum delays measured from completion of a failed wheel read. */
#define WHEEL_READ_RETRY_FIRST_MS    20U
#define WHEEL_READ_RETRY_BACKOFF_MS  50U
#define WHEEL_READ_RETRY_MAX_MS      200U
#define CMD_VEL_TIMEOUT_MS           300U
#define MICROROS_DOMAIN_ID           40U

/* Battery thresholds in XL430 raw units (0.1 V); active-high buzzer driver. */
#define BATTERY_SAMPLE_MS           100U
#define BATTERY_SAMPLE_COUNT        10U
#define BATTERY_LOW_RAW             110U
#define BATTERY_RECOVER_RAW         113U
#define BATTERY_LOW_HOLD_MS         6000U
#define BATTERY_RECOVER_HOLD_MS     3000U
#define BATTERY_MAX_GAP_MS          500U
#define BATTERY_BUZZER_PERIOD_MS    2000U
#define BATTERY_BUZZER_ON_MS        500U

#define IMU_ADDRESS_7BIT             0x68U
#define IMU_READ_PERIOD_MS           20U
#define IMU_ERROR_RETRY_MS           1000U
#define IMU_CAL_WARMUP_MS            2000U
#define IMU_CAL_SAMPLES              512U

#define IMU_FRAME_ID                 "imu_link"

/* TurtleBot3 Burger starting values. Measure and tune for the actual chassis. */
#define WHEEL_RADIUS_M               0.033f
#define WHEEL_SEPARATION_M           0.160f
#define XL430_VELOCITY_UNIT_RPM      0.229f
#define XL430_POSITION_RAD_PER_RAW   0.00153588974175501
#define XL430_VELOCITY_RAD_S_PER_RAW 0.0239808239224021

#define LEFT_WHEEL_JOINT_NAME        "wheel_left_joint"
#define RIGHT_WHEEL_JOINT_NAME       "wheel_right_joint"

/* Set a direction to -1.0f if that wheel rotates backward for positive speed. */
#define LEFT_MOTOR_DIRECTION         1.0f
#define RIGHT_MOTOR_DIRECTION        1.0f

/* URDF joint-axis sign; tune this without changing the motor command direction. */
#define LEFT_JOINT_DIRECTION         1.0
#define RIGHT_JOINT_DIRECTION        1.0


#endif
