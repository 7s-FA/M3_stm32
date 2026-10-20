# burger F411RE application — imu-ahrs-v6

STM32 owns XL430 control/readback, MPU6500/9250 reads, gyro bias calibration and
6-axis Madgwick attitude estimation. It does not compute or publish odometry/TF.
All micro-ROS endpoints now live under `/stm32` (domain 40).

- `/stm32/imu/data`: sensor_msgs/Imu, best effort, calibrated gyro + acceleration
  including gravity + quaternion; frame imu_link. Replaces the earlier /imu/data_raw.
- `/stm32/imu/status`: std_msgs/String, best effort, fw=imu-ahrs-v6, cal/attitude status.
- `/stm32/joint_states`: sensor_msgs/JointState, best effort, left then right in rad and rad/s.
  Sent only after a successful wheel read and time synchronization.
- `/stm32/cmd_vel`: geometry_msgs/Twist, forward m/s and yaw rad/s, 300ms watchdog.
- `/stm32/xl430_cmd`: std_msgs/String, motor on / motor off.
- `/stm32/battery_voltage`: std_msgs/Float32, best effort, volts, 1 s publication interval.
- Existing raw wheel command/state topics are also under /stm32.

MyApp/Src/imu_attitude.c contains the HAL/ROS-independent quaternion filter.
Calibration is RAM-only at boot; the filter uses measured sample intervals and
rejects >250ms gaps rather than integrating a stale sample across the gap.
This is the same class of 6-axis Madgwick filter as OpenCR, not identical tuning,
rate, gyro smoothing or deadband. Yaw is relative, with no absolute magnetic heading.

Raspberry Pi adapter, preserved TurtleBot3 odometry, installation and terminal-by-terminal
instructions: [raspberry_pi/README.md](../raspberry_pi/README.md).

Build: cmake --preset Debug then cmake --build --preset Debug -j4.
`burger.ioc` includes the ROS task's BelowNormal priority; worker tasks and their
static stacks are created in the preserved USER CODE sections of `freertos.c`.
Flashing and hardware validation are separate.

## FreeRTOS task ownership

| Task | Priority | Static stack | Responsibility |
| --- | --- | --- | --- |
| `defaultTask` (ROS) | BelowNormal | 12,000 bytes | All micro-ROS calls, executor, publications, discovery and teardown |
| `driveTask` | AboveNormal | 4,096 bytes | USART1/XL430, velocity and move state machines, watchdog, battery and buzzer |
| `imuTask` | Normal | 4,096 bytes | I2C1, sensor reads, calibration and attitude estimation |

The drive and IMU workers target 20 ms periods and continue during ROS discovery,
setup, publication waits and reconnect. Missed periods are skipped, not replayed
in a busy catch-up loop. Peripheral HAL calls are still blocking: the drive task's
UART response timeouts can delay the lower-priority tasks. These are not hard
real-time or end-to-end latency guarantees. Sampling/calibration timing can differ
from the previous single-task loop even though units, thresholds and algorithms
are unchanged.

`app_tasks.c` owns a statically allocated one-command queue and binary completion
semaphore. The ROS task is the sole producer and waits for command acknowledgement;
this preserves command order and avoids a backlog or silent queue overflow.
`cmd_vel` carries its original receive tick for the existing 300 ms watchdog.
The deferred torque-command handling after executor dispatch is retained.
Disconnect uses the same acknowledged path: both motor shutdown attempts finish
before ROS objects are destroyed, and no queued command survives reconnect.

Two priority-inheritance mutexes protect only copies of drive/IMU snapshots.
No hardware access, ROS call or wait for a command runs while holding them.
The service reads a coherent cached snapshot without motor I/O; sample age
continues to advance while the drive task is busy. IMU measurements are copied
to the ROS task, stamped using their acquisition tick and published at most once
per sample. Slow ROS communication can skip intermediate IMU samples rather than
accumulate an unbounded queue. Sensor state and calibration survive reconnect.
Worker readiness is signalled with a counting semaphore before ROS starts.

The ROS names, QoS, message fields, command syntax, motion limits, timeout settings,
move acknowledgements and battery thresholds are unchanged. The diagnostic
firmware tag remains `imu-ahrs-v6.7`; no Pi bridge changes are required.

The root `tests` directory contained host-only verification code and has been
removed after regression checks; it was never linked into the firmware.
After flashing, repeat the reconnect and position-move checks, verify `cmd_vel`
timeout stopping, and measure task stack margins and actual sensor/control rates.

## Battery buzzer

- PC4 (`BUZZER`) drives an active-high external driver for the 5 V active buzzer;
  do not power the buzzer directly from the STM32 GPIO. CubeMX output starts Low.
- The drive task reads left XL430 ID 1 Present Input Voltage (144, 2 bytes,
  0.1 V units) at most once per 100 ms, serialized with existing USART1 traffic.
- After 10 valid samples, a mean below 11.0 V for 6 s starts the alarm. A mean at
  least 11.3 V for 3 s clears it. The alarm is on for 500 ms every 2 s.
- A failed read or a service gap over 500 ms resets the averaging/hold window;
  an existing alarm remains latched until valid recovery readings arrive.
- Monitoring also runs while waiting for the ROS Agent and with torque off.
  Blocking motor transactions can delay sampling and GPIO transitions; ROS
  discovery no longer owns this loop. There is no independent hardware alarm timer.
- This is an audible warning only, not motor shutdown or battery protection.
  Thresholds and timings are in `MyApp/Inc/app_config.h`.
- Verify actual voltage and buzzer wiring on hardware after flashing.

## Battery voltage telemetry

Ported from [pjb/MyApp](https://github.com/7s-FA/M3_stm32/tree/pjb/MyApp).
The ROS task publishes `/stm32/battery_voltage` as `std_msgs/msg/Float32` with
best-effort QoS every 1,000 ms while connected (actual delivery can be delayed by
scheduling or communication). The value is the latest XL430 voltage in volts,
not the buzzer's moving average: raw 125 becomes 12.5 V.

The drive task copies the monitor's existing sample into its protected snapshot;
ROS performs no extra UART read. A failed read, zero value, or sample older than
500 ms suppresses publication. Age is checked again after the task handoff.
Battery publication failure alone does not stop the motors or trigger reconnect.
The publisher is cleaned up on partial initialization failure and disconnect.
Seven publisher slots are required; the bundled library has twelve.

On the host with the Agent running:

```bash
ROS_DOMAIN_ID=40 ros2 topic echo /stm32/battery_voltage --qos-reliability best_effort
```

## v6.4: on-demand drive state

The versioned notes below describe the original changes; current task ownership
and timing are described above.

The five combined/velocity/position Int32 state publishers are removed.
`/stm32/left_wheel_state` and `/stm32/right_wheel_state` remain periodic Int32
topics with the original reliable QoS, 50 ms ACK timeout and 100 ms configured
publication interval (actual rate depends on loop timing). JointState and IMU
publication, raw command topics, and the command watchdog remain unchanged.
No Pi bridge rebuild or custom ROS interface package is required. Consumers of
legacy state topics must switch to the service:

```bash
ROS_DOMAIN_ID=40 ros2 service call /stm32/get_drive_state std_srvs/srv/Trigger '{}'
```

`message` is a JSON object containing `xl430_state`,
`left_wheel_velocity_state`, `right_wheel_velocity_state`,
`left_wheel_position_state`, and `right_wheel_position_state`, preserving the
previous raw values and units. Extra fields:

- `sample_valid`: the latest wheel feedback update succeeded (also `success`).
  It does not imply torque enabled or that every motor configuration succeeded.
- `sample_tick_ms`: MCU uptime at the last successful paired wheel read, in
  milliseconds, wrapping at 32 bits; not a ROS/Unix timestamp.
- `sample_age_ms`: elapsed MCU milliseconds since that read; `4294967295` when
  no successful sample exists. Check age as well as validity before using data.
- `read_result`: latest paired-read XL430_Result_t code (0 = OK; positive enum
  error code). Before the first read it is 0 but `sample_valid` is false.

Failed reads retain the last successful raw position/velocity, mark the sample
invalid, and expose the read error. Requests only snapshot cached values and do
not initiate motor I/O or enable torque. The response uses a fixed 512-byte
buffer and a 50 ms reliable-response session timeout. Avoid high-frequency
polling: this service is for occasional diagnostics and still shares the motor
control task. Sensor/command latency is not guaranteed by this change.

Firmware build:
`cmake --preset Debug && cmake --build --preset Debug -j4`. After flashing, verify
`fw=imu-ahrs-v6.4`, absence of the five replaced state topics with a fresh ROS graph,
continued left/right wheel state publication,
service responses (including disconnected-motor feedback), continued
JointState/IMU output, and repeat the rotation latency test. Hardware validation
and flashing are separate from the build.

## v6.5: best-effort wheel feedback

`/stm32/joint_states` now uses best effort, removing its reliable ACK wait.
The configured 100 ms publication interval and successful-read/time-sync gates
are unchanged. The Pi adapter already subscribes with SensorDataQoS, so no
bridge rebuild is needed; its republished ROS-side topic QoS is unchanged.
Other subscribers to the raw STM32 topic must accept best effort. Left/right
wheel state topics remain reliable, and the drive-state service is unchanged.

Verify `fw=imu-ahrs-v6.5` and the raw publisher QoS after flashing. A successful
next encoder-position sample can cover displacement across a lost packet, but
feedback gaps still delay control and affect time-dependent estimates. Neither
best effort nor the 100 ms setting guarantees arrival every 100 ms. If gaps
persist while IMU continues, correlate the wheel state errors and service
`read_result`/`sample_age_ms`: v6.5 used a one-second retry backoff; v6.6 replaces it as described below.

## v6.6: bounded wheel-read retry backoff

After consecutive paired wheel-read failures, minimum retry delays are now
20, 50, 100, 200, 200, ... milliseconds, measured from each failed read's
completion. A successful read resets the sequence. No retry loop or sleep is
added: the application returns to its other work, and a subsequent drive update
checks whether the delay has elapsed. The first 20 ms gate also prevents two
immediate reads from the two DriveControl_Update calls in one application loop.
The retry constants are in MyApp/Inc/app_config.h.

Command processing and watchdog checks run before the retry gate. Invalid
wheel samples continue to suppress JointState; the diagnostic service retains
the previous sample age and latest read error. The XL430 response timeout stays
50 ms per response; two motor responses are read sequentially. Retry delays do
not include transaction time, other loop work, or the next publication interval,
so they are not end-to-end latency guarantees. Best-effort JointState from v6.5
and the two periodic wheel-state topics remain unchanged.

Host regression tests cover retry boundaries, repeated failures at the 200 ms
cap, time spent in the failed read, successful recovery/reset, MCU tick wrap,
and command-watchdog stopping while feedback is waiting to retry. Hardware
rotation and latency tests still need to be repeated after flashing v6.6.

## v6.7: relative pulse position moves

`/stm32/xl430_cmd` accepts `move <left pulses> <right pulses> <speed> [request ID]`.
The drive stops, executes an extended-position move, and restores zero-speed
velocity mode before reporting `DONE`. Move status is repeated on best-effort
`/stm32/move_status` and retained in the drive-state service's `move` JSON object.
Normal velocity commands are discarded during the move. Errors/cancel/disconnect
attempt to turn both motors off; a later `motor on` establishes velocity mode
before torque enable. The drive service response buffer is now 768 bytes, and
six publisher slots are required (the bundled library provides twelve).

Usage, ID/retry semantics, docking integration, bounds and hardware validation:
[상대 펄스 이동 안내](POSITION_CONTROL.md). Firmware diagnostic tag:
`fw=imu-ahrs-v6.7`. No Pi bridge rebuild is required.
