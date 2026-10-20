# M3 STM32 펌웨어 — pjb 배터리 전압 발행

STM32F411 프로젝트입니다. `pjb` 브랜치는 기존 v6.7 펌웨어에 배터리 전압
발행 기능을 추가한 `imu-ahrs-v6.7-battery` 버전입니다.

## 내려받기

```bash
git clone --branch pjb https://github.com/7s-FA/M3_stm32.git
cd M3_stm32
```

이미 저장소가 있다면 작업 내용을 커밋하거나 보관한 뒤
`git fetch origin` → `git switch --track origin/pjb`를 실행하세요.
로컬에 `pjb`가 이미 있으면 `git switch pjb`를 사용합니다.

## 변경 내용

- 기존 100ms 주기의 XL430 입력 전압 측정값을 캐시해
  `/stm32/battery_voltage`로 1Hz 발행합니다.
- 타입은 `std_msgs/msg/Float32`, 단위는 V, QoS는 best effort입니다.
- 최초 측정 전, 읽기 실패, 0값, 500ms 초과된 캐시는 발행하지 않습니다.
- 모터 UART 읽기를 추가하지 않으며 기존 주행 제어와 저전압 부저 로직을 유지합니다.
- 전압 발행 실패만으로 주행 연결을 끊지 않습니다. Agent 재연결 시 추가
  publisher도 정리하고 다시 생성합니다.
- micro-ROS publisher 슬롯은 최소 7개 필요하며 포함된 라이브러리는 12개를 제공합니다.

전압은 모터 공급 전압입니다. 배터리 잔량 %·전류·용량을 측정한 값은 아닙니다.
웹 관제용 `/burger1/battery_state` 중계는 별도 Pi 프로그램이며 이 펌웨어
저장소에는 포함되지 않습니다. STM32 전압 토픽은 아래 명령으로 직접 확인할 수 있습니다.

## 테스트와 빌드

Linux 기준 필요 도구: CMake 3.22 이상, Ninja 또는 GNU Make, Arm GNU Toolchain
(`arm-none-eabi-gcc`가 PATH에 있어야 함), 테스트용 호스트 C 컴파일러와
AddressSanitizer/UndefinedBehaviorSanitizer 런타임.
이번 펌웨어는 Arm GCC 13.2.1로 빌드했습니다.

```bash
bash tests/run_reconnection.sh
cmake --preset Debug
cmake --build --preset Debug -j4
```

결과 파일은 `build/Debug/burger.elf`입니다. 기존 micro-ROS 라이브러리와
헤더가 저장소에 포함되어 있습니다. 라이브러리를 변경해야 하는 경우의 빌드
스크립트는 `scripts/build_microros.sh`입니다.

Ninja가 없고 GNU Make가 있다면 다음 방식으로 빌드할 수 있습니다.
이번 공유 전 재빌드는 이 명령으로 검증했습니다.

```bash
cmake -S . -B build/pjb-verify -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/pjb-verify -j4
```

이 경우 결과는 `build/pjb-verify/burger.elf`이며 FLASH 169700 B,
RAM 88792 B를 사용했습니다.

2026-10-20 검증: 배터리 값 유효성·만료·복구, micro-ROS 재연결 및 23개
생성 실패 지점 정리, 주행 피드백·상대 이동·watchdog·IMU·UART 회귀 테스트 통과.
M1 실물에서는 적용 후 `imu-ahrs-v6.7-battery`와 11.70V 수신을 확인했습니다.
이 수치는 당시 측정값이며 다른 장치의 현재 전압을 뜻하지 않습니다.

## 적용 후 읽기 확인

펌웨어 기록은 로봇이 정지하고 주문이 없는 상태에서 기존 펌웨어를 백업한 뒤
보드에 맞는 기존 ST-Link/OpenOCD 절차로 진행합니다. 빌드 명령은 플래시 기록을
자동으로 수행하지 않습니다.

M1의 micro-ROS Agent와 ROS 환경이 실행된 상태에서:

```bash
ROS_DOMAIN_ID=40 ros2 topic echo /stm32/imu/status --once
ROS_DOMAIN_ID=40 ros2 topic echo /stm32/battery_voltage std_msgs/msg/Float32 --qos-reliability best_effort
```

상태의 `fw=imu-ahrs-v6.7-battery`와 정상 전압을 확인하세요.
수신 측은 전압 수신이 끊기면 마지막 값을 최신값으로 계속 표시하지 않아야 합니다.

상세 펌웨어 이력: [MyApp/README.md](MyApp/README.md).
상대 펄스 이동 설명: [MyApp/POSITION_CONTROL.md](MyApp/POSITION_CONTROL.md).
