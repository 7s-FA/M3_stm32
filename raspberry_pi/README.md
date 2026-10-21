# STM32 + Raspberry Pi micro-ROS 구성 (v6)

STM32: XL430 제어, 엔코더 읽기, 자이로 영점 보정, 6축 Madgwick quaternion.
라즈베리파이: micro-ROS Agent, ROS 인터페이스 어댑터, TurtleBot3 odometry, TF, Nav2.
OpenCR DYNAMIXEL 제어표 에뮬레이션은 사용하지 않는다.

## 이번 구현의 인터페이스

| 입력 | 출력 | 처리 |
|---|---|---|
| `/stm32/imu/data` sensor_msgs/Imu, best effort | `/burger3/imu` sensor_msgs/Imu, reliable | timestamp·SI 값 유지, frame_id=`burger3/imu_link` |
| `/stm32/joint_states` sensor_msgs/JointState | `/burger3/joint_states` 동일 형식 | 좌/우 순서 및 URDF 이름 `burger3/wheel_*_joint` 맞춤 |
| `/burger3/cmd_vel` geometry_msgs/TwistStamped | `/stm32/cmd_vel` geometry_msgs/Twist | 신선한 linear.x/angular.z 전달 |
| IMU + joint_states | `/burger3/odom` nav_msgs/Odometry, `/tf` tf2_msgs/TFMessage | 기존 ROBOTIS odometry 계산식 사용 |

`namespace:=`로 루트 토픽, `namespace:=burger3`로 위 토픽을 사용한다.
`enable_stamped_cmd_vel:=false`이면 ROS 측 cmd_vel도 Twist.
`transport_namespace` 기본 `/stm32`는 펌웨어 네임스페이스와 같아야 한다.
ROS_DOMAIN_ID=40을 Agent/터미널/펌웨어에 동일하게 사용한다.
여러 STM32를 같은 도메인에서 쓸 때는 펌웨어 transport 네임스페이스를 각각 분리해야 한다.

기존 odometry.cpp/hpp 스냅샷을 패키지에 포함하여 별도 실행한다.
wheel delta 평균 거리 + quaternion yaw 차이로 midpoint 적분하며 Z 각속도를 직접 섞지 않는다.
첫 샘플과 0.5초 초과 단절 후에는 기준값을 다시 잡는다. 단절 중 이동은 복원하지 않는다.
IMU/바퀴 stamp 차이 0.15초 초과, 수신 시 0.5초 이상 지연 데이터는 제외한다.
시간이 동기화되어야 하며, Arduino/OpenCR과 수치적으로 완전히 같은 필터 출력은 아니다.

**호환 범위:** 이번 패키지는 주행·IMU·joint_states·odom·TF 경로를 연결한다.
기존 OpenCR의 battery_state/sensor_state/magnetic_field, sound/motor_power/reset 서비스와
기타 주변장치 전체를 대체하지는 않는다. 측정되지 않은 배터리·자기장 값을 만들지 않는다.
모터 토크는 현재 `/stm32/xl430_cmd` String의 `motor on`/`motor off`로 명시적으로 제어한다.
어댑터 실행만으로 토크를 켜지 않는다. 명령 단절 시 STM32의 기존 300ms watchdog 적용.
`/burger3/reset_odometry` std_srvs/Trigger는 odometry 원점만 초기화한다.
MCU 리셋/모터 토크 전환 후에는 어댑터를 재시작하거나 odometry reset을 수행한다.

## 1. 파일 전송 및 빌드

수정된 burger 폴더 전체가 burger3의 `~/STMProject/burger`에도 있어야 한다.
개발 PC `/home/ej/STMProject/burger`를 수정한 것과 원격 Pi 파일은 별개다.
PC에서 전송하려면 (SSH 호스트 이름 burger3이 해석되는 환경):

```bash
rsync -av --exclude build --exclude .git /home/ej/STMProject/burger/ robot@burger3:~/STMProject/burger/
```

STM32는 기존 업로드 방법으로 **새 펌웨어**를 플래시한다.
빌드(ARM toolchain이 설치된 개발 PC 또는 Pi):

```bash
cd ~/STMProject/burger
cmake --preset Debug
cmake --build --preset Debug -j4
```

펌웨어 파일은 `build/Debug/burger.elf`. CubeMX 핀/클록/DMA 설정 변경 없음.

Pi에서 브리지 설치:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
mkdir -p ~/stm32_bridge_ws/src
cp -a ~/STMProject/burger/raspberry_pi/burger_stm32_bridge ~/stm32_bridge_ws/src/
cd ~/stm32_bridge_ws
colcon build --symlink-install --packages-select burger_stm32_bridge
```

## 2. 실행: 터미널 3개

기존 OpenCR용 `turtlebot3_bringup robot.launch.py`는 이 구성과 동시에 실행하지 않는다.
같은 USB 포트를 열거나 odom/TF를 중복 발행하기 때문이다.
이 launch는 어댑터, odometry와 robot_state_publisher를 실행한다. LiDAR/Nav2는 별도다.

### 터미널 1 — Agent (먼저 실행, 계속 켜 둠)

STM32 USART2와 Agent는 모두 **921600bps (8N1)**를 사용한다.
기존 `imu-agent-debug`가 이미 `-b 921600`으로 생성된 경우 재사용:

```bash
docker stop imu-agent-debug
docker start -ai imu-agent-debug
```

기존 컨테이너가 `-b 115200`으로 생성되었다면, 재시작만으로 속도가 바뀌지 않는다.
최초 전환 시 기존 컨테이너를 정지하고 이름을 변경해 보관한다
(`imu-agent-debug-115200`이라는 이름이 아직 없는 경우):

```bash
docker stop imu-agent-debug
docker rename imu-agent-debug imu-agent-debug-115200
```

기존 컨테이너가 없거나 위 전환을 마쳤으면 새로 생성:

```bash
docker run -it --name imu-agent-debug --net=host --privileged \
  -e ROS_DOMAIN_ID=40 -v /dev:/dev microros/micro-ros-agent:jazzy \
  serial --dev /dev/ttyACM0 -b 921600 -v4
```

Agent 실행 후 STM32 리셋. 새 진단 표시는 `fw=imu-ahrs-v6`.
기존 펌웨어의 Agent 재접속 상태기계는 이번 변경 대상이 아니므로 Agent 재시작 시 MCU도 리셋.

### 터미널 2 — ROS 드라이버 및 odometry

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/stm32_bridge_ws/install/setup.bash
export ROS_DOMAIN_ID=40
export TURTLEBOT3_MODEL=burger
timeout 10s ros2 topic echo /stm32/imu/status std_msgs/msg/String \
  --qos-reliability best_effort --full-length --once
```

기존 별도 robot_state_publisher를 사용한다면 `state_publisher:=false` 추가.
기본 모델 URDF의 imu_link transform을 사용한다. 실제 IMU 위치와 맞는지는 별도 확인 대상.

### 터미널 3 — 데이터 확인 (명령마다 Ctrl+C 후 다음 명령)

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
ros2 topic echo /stm32/imu/status std_msgs/msg/String --qos-reliability best_effort
```

`cal=READY`, `attitude=1`, 증가하는 samples 확인.

```bash
ros2 topic echo /burger3/imu sensor_msgs/msg/Imu --once
ros2 topic hz /burger3/joint_states
ros2 topic hz /burger3/odom
ros2 run tf2_ros tf2_echo burger3/odom burger3/base_footprint
```

IMU는 orientation_covariance[0]=0(추정치 제공, 공분산 미측정), 정규화된 quaternion.
자기장 없이 계산하므로 yaw는 시작 방향 기준 상대각이며 시간이 지나면 drift 가능.
가속도는 중력 포함. 모터 읽기가 안 되면 joint_states/odom은 나오지 않는다.
IMU만 나오는 상황과 바퀴 통신 실패를 구분해야 한다.
현재 실제 발행 주기는 UART 처리량과 모터 응답에 영향받으며, 200Hz OpenCR 성능과 같다고 보장하지 않는다.

## 3. 검증 이후

주행 명령은 `/burger3/cmd_vel` TwistStamped에 연결한다.
Nav2 기존 namespace/remapping 및 LiDAR launch는 실제 burger3 실행 구성에 맞춰 연결해야 한다.
이 단계 전에 IMU·wheel 데이터의 방향/단위, odom/TF 단일 발행, 시간 동기화를 확인한다.
이번 구현에서 실물 로봇을 구동하거나 펌웨어를 업로드하지 않았다.

## 개발 테스트

`tests/integration.py`는 도메인 221에서 가상 IMU/바퀴를 주입하고 실제 DDS로
네임스페이스, QoS, quaternion-yaw odometry, TF, TwistStamped 변환,
오래된 명령 거부, 초기 기준값, 센서 단절, reset_odometry를 검사한다.
하드웨어 Agent 없이 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/stm32_bridge_ws/install/setup.bash
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 python3 \
  ~/stm32_bridge_ws/src/burger_stm32_bridge/tests/integration.py
```

## v6.1 바퀴 피드백 수정

v6에서는 velocity_ready 이후에만 엔코더를 읽어, 부팅 후 토크 OFF 상태에서
joint_states/odom/TF가 없을 수 있었다. v6.1은 토크·속도 명령과 독립적으로
엔코더를 읽는다. 모터 전원과 통신은 필요하지만 motor on 명령은 필요하지 않다.
읽기 실패 시 joint_states를 발행하지 않고 1초 후 재시도한다.
펌웨어 진단 fw=imu-ahrs-v6.1로 업로드 여부 확인. Pi 브리지 재빌드는 필요 없다.

## v6.2 시간 동기화 수정

MCU epoch가 Pi보다 13초 앞선 실물 로그로 인해 어댑터 freshness 검사에서
IMU/JointState가 제외되는 문제가 확인되었다. v6.2에서는 최초 동기화 후에도
모터 비활성 상태에서 2초마다 최대 100ms로 Agent 재동기화를 요청한다.
기존 time_synced=1은 한 번 성공했다는 표시이며 실제 시간 오차의 보장이 아니다.
Pi 어댑터는 시간 차이로 메시지를 제외할 때 5초 간격 경고를 남긴다.
허용 시간창이나 메시지 원래 timestamp는 바꾸지 않는다.
장시간 모터 활성 중 drift/호스트 시각 변경 대응은 별도 검증·보완 대상이다.
시간 차이가 생긴 정확한 원인(보드 clock drift 또는 Pi 시간 보정)은 이 로그만으로 확정 불가.

## v6.4 상태 조회 서비스

`left_wheel_state`와 `right_wheel_state`는 기존 주기(설정 100ms)의 reliable
Int32 토픽으로 유지한다. 나머지 5개 토픽의 주기 발행을 제거하고 `/stm32/get_drive_state`
(`std_srvs/srv/Trigger`) 요청에 한 번 응답하도록 변경했다.
브리지 빌드와 joint_states/IMU 연결은 그대로 사용할 수 있다.
서비스로 전환한 5개 상태 토픽의 구독 도구는 서비스 호출로 변경해야 한다.

```bash
ROS_DOMAIN_ID=40 ros2 service call /stm32/get_drive_state std_srvs/srv/Trigger '{}'
```

응답 `message`는 나머지 5개 값과 측정 시각·유효 여부·읽기 오류를 담은 JSON이다.
필드 의미와 검증 절차는 [펌웨어 문서](../MyApp/README.md#v64-on-demand-drive-state)를 따른다.
요청 시 모터를 새로 읽지 않으며, 필요할 때만 조회한다.

## v6.5 joint_states QoS

`/stm32/joint_states`는 IMU처럼 best effort로 발행한다. 브리지는 이미
SensorDataQoS로 구독하므로 재빌드가 필요 없다. raw 토픽을 직접 구독하는
도구도 best effort를 사용해야 한다. 브리지 출력 QoS와 설정 발행 주기는
그대로다. 공백이 계속되면 양쪽 상태 오류와 서비스의 읽기 결과·샘플
경과 시간을 확인한다. v6.5의 1초 재시도는 v6.6에서 아래처럼 변경했다.

## v6.6 모터 읽기 재시도

읽기 실패가 이어지면 실패 완료 시점부터 20→50→100→200ms를 기다리고,
이후에는 200ms로 유지한다. 한 번 성공하면 다음 실패의 대기는 다시 20ms다.
대기 중에도 명령 처리와 정지 감시를 수행하며, 별도 sleep은 사용하지 않는다.
모터 응답 타임아웃(응답당 50ms)과 통신·발행 시간은 별도로 소요된다.
펌웨어 식별자는 `fw=imu-ahrs-v6.6`이며, 업로드 후 실제 공백을 재측정해야 한다.
