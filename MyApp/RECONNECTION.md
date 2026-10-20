# Agent 자동 연결과 재연결

STM이 먼저 부팅해도 Agent를 기다렸다가 ROS 객체를 생성한다.
연결 중에는 200ms 간격의 ping(최대 50ms 대기), executor 오류 및 reliable
발행 실패로 연결 이상을 감지한다. 빠른 Agent 재시작은 ping이 성공해도
이전 세션의 발행이 실패할 수 있으므로 발행 실패도 복구 조건이다.
Reliable 발행의 응답 대기는 50ms로 제한하며 첫 실패에서 정리 단계로 간다.
이 수치는 통신 호출의 설정값이며 실제 모터 정지 시간 보증은 아니다.

오류 발생 시 대기 중인 명령을 폐기하고 모터 속도 0/torque off를 요청한 뒤
생성된 ROS 객체를 해제한다. 초기화 중간 실패도 재시도한다.
복구 후 시간 동기화를 다시 수행한다. IMU 센서 초기화와 보정값은 유지한다.
모터는 자동 활성화하지 않는다. 다시 주행하려면 새 `motor on`과 속도 명령이 필요하다.
물리적으로 움직인 통신 단절 구간의 IMU 자세/odom 정확도는 보장하지 않는다.

UART DMA 전송 구현은 `MyApp/Src/microros_transport.c`를 사용한다.
다시 열 때 DMA 읽기 위치를 초기화하며 외부 micro-ROS 유틸리티 교체에 영향을 받지 않는다.

## 검증

```bash
bash tests/run_reconnection.sh
cmake --preset Debug
cmake --build --preset Debug -j4
```

호스트 테스트는 실제 애플리케이션 루프에 가짜 ROS/HAL 응답을 주입한다.
Agent 부재, 50회 세션 생성, ping/executor/발행 실패, 23개 생성 단계 실패,
부분 support 초기화 실패, 명령 폐기, IMU 보정값 유지 및 DMA 재열기를 확인한다.
실제 micro-ROS 라이브러리의 반복 연결 시 메모리 사용과 보드 동작은 실물 확인 대상이다.

## 플래시 후 실물 확인

1. Agent가 꺼진 상태로 STM을 부팅한다. Agent를 켜고 Reset 없이
   `/stm32/imu/status`와 `/stm32/joint_states`가 나오는지 확인한다.
2. Agent를 중지했다가 다시 시작한다. STM Reset 없이 토픽이 복구돼야 한다.
3. Agent를 빠르게 재시작하는 경우도 확인한다. 기존 세션 오류 이후 복구돼야 한다.
4. 재연결만으로 모터가 활성화되지 않는지, 새 `motor on`과 속도 명령 후에만
   구동되는지 확인한다. 여러 번 반복해 멈춤이나 메모리 부족이 없는지 확인한다.

Docker 권한, IMU 보정 판정 조건 및 Pi odometry 설정은 이 변경의 대상이 아니다.
