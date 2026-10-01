# STM32 센서·모터 펌웨어

STM32는 XL430 제어, 명령 타임아웃/정지, 엔코더 읽기, IMU 읽기와 자이로 보정을 수행한다.
Raspberry Pi가 로봇 위치/방향 odometry, TF, Nav2를 담당한다.

이번 정리에서 바퀴 odometry, gyro Z heading 적분, IMU/바퀴 결합,
Odometry/TF 메시지 변환, /odom 및 /tf publisher를 삭제했다.
센서값의 단위 변환, 바퀴 속도 명령 변환, JointState 출력은 유지한다.

현재 전송은 micro-ROS이며 OpenCR DYNAMIXEL 제어 테이블 호환은 아직 구현되지 않았다.
IMU quaternion 추정도 아직 없으므로 orientation_covariance[0]=-1을 유지한다.
OpenCR 원본은 IMU 필터로 quaternion을 계산하지만 이번 작업은 불필요한 계산 삭제만 수행했다.

## 유지한 ROS 인터페이스

구독: /cmd_vel, /xl430_cmd, /left_wheel_velocity_cmd, /right_wheel_velocity_cmd.
발행: /joint_states, /xl430_state, 좌우 wheel_state/velocity_state/position_state,
/imu/data_raw, /imu/status. 중복 /xl430_velocity_state는 이전처럼 비활성이다.
진단 버전은 fw=imu-only-v5이며 yaw 입력 선택 표시는 삭제했다.

## 빌드

```bash
cmake --preset Debug
cmake --build --preset Debug -j4
```

IMU 보정/테스트는 [IMU.md](IMU.md) 참고. 해당 문서의 과거 버전 문자열 대신
현재 진단에서는 imu-only-v5가 표시된다. 센서/모터 실기 테스트는 업로드 후 필요하다.
