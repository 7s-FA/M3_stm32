# XL430 상대 펄스 이동 — v6.7

F411 펌웨어가 모터의 확장 위치 모드(4)로 상대 이동을 실행하고, 도착 후
속도 모드(1)로 복귀합니다. 목표 도착과 모드 복귀에 성공해야 `DONE`이 됩니다.
라즈베리파이 브리지의 수정이나 새 ROS 메시지 패키지는 필요하지 않습니다.
상위 도킹 노드의 `DOCKED` 전환은 아래 상태 인터페이스에 연결해야 합니다.

## 빌드와 실행 예

```bash
cmake --preset Debug
cmake --build --preset Debug -j4
```

산출물: `build/Debug/burger.elf`. 펌웨어를 보드에 기록하고 Agent를 연결한 뒤
`/stm32/imu/status`에서 `fw=imu-ahrs-v6.7`을 확인합니다. 코드 빌드와 호스트
테스트는 보드 기록·실제 모터 검증을 대신하지 않습니다.

터미널 1 — 상태 확인:

```bash
ROS_DOMAIN_ID=40 ros2 topic echo /stm32/move_status std_msgs/msg/String --qos-reliability best_effort
```

터미널 2 — 모터 활성화 후 약 10 mm 전진:

```bash
ROS_DOMAIN_ID=40 ros2 topic pub --once /stm32/xl430_cmd std_msgs/msg/String "{data: 'motor on'}"
ROS_DOMAIN_ID=40 ros2 topic pub --once /stm32/xl430_cmd std_msgs/msg/String "{data: 'move 198 198 20 1'}"
```

명령 형식:

```text
move <왼쪽 상대 펄스> <오른쪽 상대 펄스> <프로파일 속도> [요청 ID]
```

- 펄스 범위: 바퀴마다 -40960~40960. 양수는 기존 `*_MOTOR_DIRECTION` 설정을
  적용한 전진 방향, 음수는 후진입니다. 모터에 설정된 Drive Mode 방향은 유지합니다.
- 속도: raw 1~100. raw 20은 4.58 rpm, 반지름 33 mm 기준 약 15.8 mm/s입니다.
  0은 허용하지 않습니다. 두 바퀴에 같은 최고 속도·가속도를 설정하므로 서로 다른
  펄스를 요청하면 이동 시간이 다를 수 있습니다. 양쪽 모두 도착해야 완료됩니다.
- 요청 ID: 1~4294967295의 증가하는 정수. 예제 다음 새 이동은 ID 2를 사용합니다.
  생략하면 마지막 승인 ID + 1을 배정합니다. 자동 도킹에서는 반드시 명시하세요.
- 같은 마지막 ID와 같은 인자 재전송은 `DUPLICATE`로 응답하고 재실행하지 않습니다.
  같은 ID의 다른 인자는 `ID_CONFLICT`, 이전 ID는 `STALE_ID`입니다.
- ID와 결과는 ROS Agent 재연결 동안 유지되지만 MCU 재부팅 시 초기화됩니다.
  보드 재부팅 후 이전 이동을 자동 재전송하지 마세요. 상위 제어기가 상태를 다시
  확인해야 합니다. 여러 명령 송신자는 하나의 ID 관리자로 통합해야 합니다.
- 양쪽 모터가 활성화된 상태에서만 접수합니다. `motor on`도 속도 0의 속도 모드를
  먼저 설정하므로 취소된 위치 목표를 다시 실행하지 않습니다.

수동 취소:

```bash
ROS_DOMAIN_ID=40 ros2 topic pub --once /stm32/xl430_cmd std_msgs/msg/String "{data: 'motor off'}"
```

## 상태와 도킹 연동

`/stm32/move_status`는 `std_msgs/String`의 JSON이며 best effort로 기존 상태
발행 주기(설정 100 ms)에 맞춰 반복 발행합니다. 실제 주기는 작업 지연에 영향을
받습니다. 마지막 결과를 다음 승인 이동까지 유지하므로 단발 완료 이벤트 유실에
의존하지 않습니다. 명령 거절 응답은 마지막 한 건만 유지합니다.

```json
{"id":1,"state":"DONE","reason":"none","request_id":1,"ack":"ACCEPTED"}
```

| 필드 | 의미 |
|---|---|
| `id` | 현재 또는 마지막 승인 이동 ID |
| `state` | `IDLE`, `STOPPING`, `MOVING`, `DONE`, `ERROR`, `CANCELED` |
| `reason` | 실행 실패·취소 이유. 정상은 `none` |
| `request_id` | 마지막 명령의 명시 ID 또는 승인 시 자동 배정 ID. 해석 불가 시 0 |
| `ack` | 마지막 요청의 접수·거절 결과. 실행 결과와 별개 |

`ack`에는 `ACCEPTED`, `DUPLICATE`, `BUSY`, `DISABLED`, `INVALID`,
`ID_CONFLICT`, `STALE_ID`, `ID_EXHAUSTED` 등이 들어갑니다. 다른 명령을 거절해도
현재 이동의 `id/state/reason`은 덮어쓰지 않습니다. `ERROR`의 `reason`은
`feedback`, `timeout`, `position_setup`, `position_enable`, `goal_range`,
`goal_write`, `restore_mode`, `torque_lost` 등으로 원인 단계를 나타냅니다.

기존 조회 서비스에서도 `message` JSON의 `move` 객체로 같은 상태를 읽습니다.
서비스 `success`는 기존과 같이 바퀴 피드백 유효성이지 이동 성공 여부가 아닙니다.

```bash
ROS_DOMAIN_ID=40 ros2 service call /stm32/get_drive_state std_srvs/srv/Trigger '{}'
```

상위 도킹 노드의 처리 순서:

1. 일반 `cmd_vel` 송신을 멈추고 속도 0으로 정지시킵니다.
2. 정지 후 남은 직진 거리를 다시 측정하고 `N = round(distance_mm * 4096 /
   (2 * pi * 33))`으로 변환합니다. 이미 이동 중인 상태에서 계산한 남은 거리를
   그대로 보내면 펌웨어의 정지 과정에서 진행한 거리만큼 목표가 달라질 수 있습니다.
3. 증가하는 요청 ID로 `move N N speed id`를 한 번 보냅니다.
4. 자기 요청 ID의 `state == DONE`을 확인합니다. `ACCEPTED`만으로 완료 처리하지
   않습니다. 오류·취소·접수 거절 시 `DOCKED`로 전환하지 않습니다.
5. 도킹 접촉이나 충전 성립이 완료 조건이면 해당 센서도 확인한 뒤 `DOCKED`로
   전환합니다. `DONE`은 바퀴 이동 완료이며 충전·접촉 확인을 의미하지 않습니다.

## 실행과 보호 동작

- `STOPPING`: 양쪽 속도 0을 쓰고 정지를 확인합니다. 3초 내 정지하지 못하면 오류.
- 양쪽 토크 OFF → 모드 4 → 속도 기반 프로파일 설정 → 현재 위치 재측정 →
  현재 위치를 초기 목표로 기록 → 토크 ON/확인 → 현재 위치 재측정 → 상대 목표
  계산 → 양쪽 Goal Position Sync Write 순서로 실행합니다.
- Profile Acceleration은 raw 10. 모드 전환 때 초기화되는 프로파일을 다시 설정하고,
  Drive Mode의 시간 기반 프로파일 및 Goal 갱신 시 자동 토크 켜기를 비활성화합니다.
- `MOVING` 동안 `cmd_vel`과 바퀴별 속도 명령을 버립니다. 새 `move`는 `BUSY`입니다.
  완료 후 과거 속도 명령은 재생하지 않지만, 완료 이후 새로 들어오는 `cmd_vel`은
  정상 실행됩니다. 따라서 상위 노드에서도 일반 주행 송신을 중지해야 합니다.
- 도착 기준: 양쪽 위치 오차 ±2펄스, 실제 속도 raw ±1 이내가 200 ms 유지.
  토크 상태까지 확인하고 속도 모드·속도 0으로 복귀한 후 `DONE`을 발행합니다.
- 이동 제한 시간은 60초입니다. 긴 거리·낮은 속도 조합도 이 제한을 적용받습니다.
  관련 상수는 `MyApp/Inc/app_config.h`에 있습니다.
- 피드백 읽기 실패, 모드 전환 실패, 목표 범위 초과 등은 `ERROR`로 처리하고 양쪽
  토크 OFF를 시도합니다. 정상 종료만 속도 모드로 자동 복귀합니다. 오류·취소 후에는
  원인을 확인하고 `motor on` 및 새 ID로 다시 시작합니다.
- `motor off` 또는 Agent 연결 상실은 이동 취소와 양쪽 토크 OFF를 시도합니다.
  버스 자체 단절로 OFF 명령도 전달되지 않는 경우 실제 토크 해제는 보장할 수 없습니다.
  모터 Bus Watchdog 1초 설정도 유지됩니다. watchdog 정지와 토크 해제는 다릅니다.
- 모드 전환 직전·직후 엔코더 기준을 보정해 JointState의 좌표 점프를 억제합니다.
  토크 OFF 구간의 실제 미세 이동과 카운터 초기화를 완전히 구분하지는 못합니다.
- 임의 음수 위치 패킷에 필요한 Protocol 2.0 송수신 byte stuffing을 처리합니다.

## 실제 장비 검증과 정확도

처음에는 바퀴를 띄워 전진·후진 방향, 양쪽 움직임, 취소, 모드 복귀를 확인한 뒤
바닥에서 작은 거리로 시험하세요. 바닥 시험에서는 반복 오차, 토크 OFF 때 밀림,
장애물로 목표 미도달 시 timeout, 일반 주행 재개를 확인해야 합니다.

현재 반지름 설정에서 1펄스는 약 0.0506 mm입니다. ±2펄스 판정은 엔코더 기준이며
차체의 0.1 mm 정지 정확도를 보장하지 않습니다. 슬립·타이어 변형·기어 유격·바퀴
반지름 보정이 영향을 줍니다. 속도 모드로 복귀하면 목표 위치를 계속 유지하지 않습니다.

펌웨어 빌드:

```bash
cmake --preset Debug
cmake --build --preset Debug -j4
```

호스트 검증 전용 `tests` 폴더는 회귀 검증 후 삭제했다. 실제 모터 제어 정확도와
기구 거동은 위 실물 검증 절차로 확인한다.

참고: [XL430 제어 테이블](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/),
[Protocol 2.0 패킷 처리](https://emanual.robotis.com/docs/en/dxl/protocol2/#packet-process).
