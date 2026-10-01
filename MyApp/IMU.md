# MPU9250 초기 연결 — NUCLEO-F411RE

## 배선 / CubeMX

- PB8 (D15) → SCL, PB9 (D14) → SDA, GND 공통.
- 모듈의 3.3V 입력 지원 여부를 확인한 뒤 전원 연결. 칩 I/O에 5V 금지.
- AD0 → GND: 센서 주소 0x68. CS → 3.3V: I2C 모드.
- SCL/SDA에는 3.3V 풀업 필요. 모듈에 없다면 각각 4.7kΩ부터 시작.
- I2C1: 100 kHz, 7-bit, Primary Slave Address **0**.
  이 항목은 STM32 자신의 주소이며 MPU9250 주소가 아니다.
- PB8/PB9: AF4, Open Drain, No pull (외부 풀업 사용).
- INT 배선, I2C IRQ/DMA 및 새 태스크는 이번 구현에 필요하지 않다.
- 이번 변경은 기존 100 kHz를 유지한다. `.ioc`와 생성된 `i2c.c`의
  OwnAddress1만 0으로 맞췄다.

## 파일과 실행 흐름

- `MyApp/Src/mpu9250.c`: HAL 드라이버. 0x75에서 0x71(MPU9250) 또는 0x70(MPU6500) 확인 후 초기화.
  ±4g / ±500 deg/s, 자이로 출력 100 Hz, 저역통과필터 설정.
  INT_STATUS와 가속도·온도·자이로를 15바이트 연속 읽기.
- `MyApp/Src/imu.c`: I2C1 연결, 읽기 주기, 상태 기록, ROS 메시지 발행.
- `MyApp/Src/rtos.c`: 모터 초기화 전 `IMU_InitSensor()`, 노드 생성 후
  `IMU_InitPublisher()`, 모터 갱신 뒤 `IMU_Update()` 호출.
- `MyApp/Inc/app_config.h`: 센서 주소, 읽기 주기, 오류 재시도 간격, frame_id.
- 상위 `CMakeLists.txt`에 두 C 파일 등록. CubeMX 재생성 후에도 유지할 사용자 설정.

센서 초기화는 부팅 시 약 240 ms 대기한다. 초기화 실패는 모터/ROS 시작을
막지 않으며, 배선/전원을 수정한 뒤 STM32를 리셋하여 재시도한다.
읽기 오류 시 오래된 값을 발행하지 않고 1초 후 다시 읽는다. 전원 재인가로
센서 설정이 초기화된 경우 STM32도 리셋해야 한다. 자동 재초기화는 구현하지 않았다.
HAL 호출의 timeout은 5 ms지만 HAL 내부 BUSY 대기와 태스크 선점으로 실제
호출 시간은 더 길 수 있다. 모터 운전 중 응답 지연은 실기에서 확인해야 한다.

## ROS 인터페이스

- `/imu/data_raw`: `sensor_msgs/msg/Imu`, best-effort, `imu_link`.
- 최소 읽기 간격 20 ms. 기존 루프의 delay/모터/ROS 처리 때문에 실제 발행률은
  50 Hz 이하이며 정확한 50/100 Hz를 보장하지 않는다. FIFO는 사용하지 않는다.
- 가속도: m/s² (중력 포함), 각속도: rad/s (시작 시 측정한 자이로 바이어스 제거).
  +X 앞, +Y 왼쪽, +Z 위 장착에 맞춰 센서 XYZ를 그대로 사용한다.
- 자세 추정 없음: orientation_covariance[0] = -1.
- 가속도/각속도 covariance 0: 미측정/알 수 없음. 영분산을 의미하지 않는다.
- ROS 시간 동기화가 된 후 새 데이터만 발행. timestamp는 읽기 완료 시각 근사값.
- 지자기 AK8963, 가속도 보정, 자세 필터, odom 융합은 포함하지 않는다.
- imu_link → 로봇 몸체의 장착 위치/방향은 실제 장착 후 URDF에 반영해야 한다.
  이 드라이버에서 임의로 축을 뒤집거나 TF를 추가하지 않는다.
- 기존 11개 publisher에 IMU 1개 추가. 포함된 라이브러리는 용량 12이며 재생성 불필요.

## 빌드 및 확인

프로젝트 루트에서:

```bash
cmake --preset Debug
cmake --build --preset Debug -j4
```

펌웨어 업로드 및 micro-ROS Agent 연결 후 ROS 환경을 source한 터미널에서:

```bash
export ROS_DOMAIN_ID=40
ros2 topic echo /imu/data_raw --qos-reliability best_effort
ros2 topic hz /imu/data_raw
```

정지 상태에서 가속도 벡터 크기가 약 9.81 m/s², 각속도는 작은 바이어스가
있는 0 근처인지 확인한다. Z축 방향에 따라 Z 중력값의 부호는 달라진다.
이는 배선/기본 동작 확인이며 보정 완료를 의미하지 않는다.

디버거의 `imu_status`에서 `who_am_i=0x71` 또는 `0x70`, `initialized`, `sample_valid`,
`last_result`, `last_hal_status`, `samples`, `read_errors`, `publish_errors`를 확인.
`sample_valid`는 마지막 시도의 결과이며 시간 경과에 따른 freshness 판정은 아니다.
다른 태스크가 읽는다면 동기화와 `sample.tick_ms`를 이용한 오래된 값 검사가 필요하다.

호스트 드라이버 테스트:

```bash
cc -std=c11 -Wall -Wextra -Werror -pedantic -fsanitize=undefined,address \
  -Itests/imu_stubs -IMyApp/Inc MyApp/Src/mpu9250.c tests/test_mpu9250.c \
  -lm -o /tmp/test_burger_mpu9250
ASAN_OPTIONS=detect_leaks=0 /tmp/test_burger_mpu9250

cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined,address \
  -Itests/imu_stubs -IMyApp/Inc \
  -Icmake/microros/libmicroros/microros_include \
  MyApp/Src/imu.c MyApp/Src/gyro_calibration.c tests/test_imu.c -o /tmp/test_burger_imu
ASAN_OPTIONS=detect_leaks=0 /tmp/test_burger_imu
```

테스트 실행 환경의 ptrace 제약 때문에 LeakSanitizer만 끈다.
AddressSanitizer/UndefinedBehaviorSanitizer 검사는 활성화되어 있다.

센서 실물에서의 ACK, 값, 모터와 동시 운전은 별도 확인이 필요하다.

## 디버거 없이 진단 (imu-cal-v3)

진단 서비스를 제거하고 `/imu/status` std_msgs/msg/String을 best-effort로
1초마다 발행한다. 센서 실패/시간 미동기화/실행기 오류에도 루프가 돌면 발행한다.
기존 중복 호환 토픽 `/xl430_velocity_state`는 publisher 슬롯 확보를 위해
임시 비활성화했다. 같은 값을 `/right_wheel_velocity_state`에서 받을 수 있다.
총 publisher 수는 기존 라이브러리 용량인 12개를 유지한다.

```bash
ros2 topic echo /imu/status std_msgs/msg/String --qos-reliability best_effort
```

`fw=imu-cal-v3`로 이번 펌웨어 적용 여부를 확인한다.
initialized=0이면 초기화 실패, who_am_i는 MPU9250이면 0x71, MPU6500이면 0x70이다.
result=I2C_ERROR일 때 hal은 0=OK, 1=ERROR, 2=BUSY, 3=TIMEOUT이다.
samples가 증가하는데 time_synced=0이면 시간 동기화로 IMU 발행이 보류된다.
spin_errors와 last_spin_error는 실행기의 timeout 이외 오류를 기록한다.
sample_age_ms=4294967295는 성공한 샘플이 아직 없다는 뜻이다.
서비스 호출 명령은 더 이상 사용하지 않는다.

## MPU6500 호환

실물에서 0x70이 관측되어 MPU6500의 가속도·자이로·온도 레지스터를 지원한다.
0x70/0x71만 허용하고 다른 ID는 계속 거부한다. I2C 주소는 0x68 그대로다.
모듈의 정품 여부나 별도 지자기 부품 유무는 식별값만으로 확정하지 않는다.
진단의 model은 WHO_AM_I 기반 판정이다. 지자기 기능은 구현하지 않았다.

## 시작 시 자이로 보정

`gyro_calibration.c`에서 2초 준비 후 최소 100개 샘플과 최소 5초 길이의
유효 구간을 모두 확보하면 바이어스를 결정한다. 10 Hz이면 총 약 12초,
50 Hz이면 약 7초이며 ROS 초기화 시간이나 재시도에 따라 늘어날 수 있다.
루프를 막는 대기 없이 진행하며 cal=READY 전에는 /imu/data_raw를 발행하지 않는다.

진단 예: `fw=imu-cal-v3 cal=READY n=100/100 bias_mrad_s=[-27,99,-24]`.
보정 상태는 WARMUP → COLLECTING → READY 순서다. n이 100을 넘더라도
5초 조건을 만족할 때까지 수집할 수 있다. bias_mrad_s는 rad/s의 1000배를
정수로 표시한 값이다. 실제 보정에는 반올림하지 않은 float 값을 사용한다.

I2C 오류, 샘플 간 500 ms 초과 공백, 비정상값, 큰 각속도/가속도 변화 또는
자이로 표준편차 0.01 rad/s 초과 시 미완료 구간을 폐기한다. 이 검사는
일정한 느린 회전을 바이어스와 완전히 구분하지 못한다. 완료 후에는
바이어스를 고정하여 실제 회전을 계속 보정값으로 흡수하지 않는다.

바이어스는 RAM에만 저장하며 리셋마다 다시 측정한다. 온도 변화에 따른
장기 드리프트 보상은 아니다. 가속도 중력/방향, quaternion, odom은 바꾸지 않는다.
raw sample은 imu_status.sample에 남기고 ROS 각속도에만 바이어스를 뺀다.

호스트 보정 테스트:
```bash
cc -std=c11 -Wall -Wextra -Werror -IMyApp/Inc \
  MyApp/Src/gyro_calibration.c tests/test_gyro_calibration.c -lm -o /tmp/test_gyro_cal
/tmp/test_gyro_cal
```
