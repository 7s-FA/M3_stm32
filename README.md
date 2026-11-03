# M3_stm32 · 운반 로봇 하위 제어 펌웨어

운반 로봇의 바퀴 모터, IMU, 배터리를 제어하는 STM32 펌웨어입니다. 로봇의 상위 컴퓨터가 속도를 보내면 STM32가 모터를 구동하고 센서 값을 반환합니다. 경로 계획과 지도 위치 계산은 상위 컴퓨터가 담당합니다.

## 주요 기능

| 기능 | 내용 |
|---|---|
| 주행 | XL430 바퀴 모터 속도 제어·피드백 |
| 자세 측정 | MPU6500/9250 IMU 읽기와 자세 추정 |
| 상태 확인 | 배터리 전압·모터·IMU 상태 보고 |
| 연결 복구 | 통신 상태 감시와 오류 진단 |

## 통신과 코드

micro-ROS로 상위 컴퓨터와 연결합니다. 대표 입력은 `/stm32/cmd_vel`, 출력은 `/stm32/joint_states`, `/stm32/imu/data`, `/stm32/battery_voltage`입니다.

- [MyApp](MyApp): 펌웨어 핵심 코드와 상세 설명
- [raspberry_pi](raspberry_pi): 로봇 상위 컴퓨터 연결 안내
- [tests](tests): 하드웨어 기능의 단위 테스트

빌드 환경을 준비한 뒤 저장소 루트에서 실행합니다.

```bash
git submodule update --init --recursive
cmake --preset Debug
cmake --build --preset Debug -j4
```

`host_total`에는 관제 PC의 펌웨어 소스를 보관했습니다. 이번 게시에서 보드 재플래시는 하지 않았습니다. [소스 출처와 확인 범위](HOST_TOTAL.md)
