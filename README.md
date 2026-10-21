# host_total · M1 STM32 IMU 복구 (2026-10-21)

기준은 `main`의 `716f271`입니다. 배터리 전압 발행과 주행/IMU/ROS FreeRTOS 태스크 분리는 이미 기준 커밋에 포함돼 있습니다. 이 브랜치는 현장 IMU 복구 변경과 회귀 테스트를 추가합니다.

## 추가·수정

- `MyApp/Src/imu_bus.c`, `MyApp/Inc/imu_bus.h`: I2C 버스 복구와 RTOS 양보 대기. SCL/SDA를 강제로 HIGH 출력하지 않고 open-drain으로 해제 상태를 확인.
- `imu.c`: 부팅 초기화 실패와 I2C 읽기 오류에서 제한된 간격으로 재시도. 정상 복구 시 기존 보정값/yaw를 유지하고 긴 간격 샘플 적분은 기존 보호에 따름.
- `mpu9250.c`: 센서 초기화 대기를 `HAL_Delay`에서 `IMUBus_Delay`로 바꿔 대기 중 다른 태스크 실행 허용.
- `imu_diagnostics.c`, `imu.h`: 복구 시도/성공 횟수 및 `imu-v6.8-recovery` 진단 표기.
- `CMakeLists.txt`: 새 버스 복구 파일을 빌드에 포함.
- `tests/`: IMU·I2C·배터리·주행 피드백·위치 이동·재연결·전송 복구 회귀 검사와 HAL 스텁.

## 빌드·검증

```bash
git submodule update --init --recursive
cmake --preset Debug
cmake --build --preset Debug -j4
bash tests/run_reconnection.sh
```

회귀 테스트의 micro-ROS 헤더는 프로젝트 빌드 환경이 필요합니다. [애플리케이션 설명](MyApp/README.md)과 [Pi 어댑터 안내](raspberry_pi/README.md)를 함께 확인하세요.

M1에 보관된 `burger-recovery.elf`와 호스트 빌드의 SHA-256이 일치함을 확인했지만 이번 게시에서는 MCU 플래시를 읽거나 쓰지 않았습니다. 빌드 파일·백업 플래시·실행 로그는 올리지 않습니다. 펌웨어 적용·토크·주행 확인은 장치 정지 상태에서 별도로 수행해야 합니다.

## 게시 전 확인

tests/run_reconnection.sh의 IMU 버스·센서·자세·재연결·주행·배터리·위치·XL430 회귀 검사 모두 통과.
