#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
bin_dir=$(mktemp -d /tmp/burger-reconnect-tests.XXXXXX)
flags=(-std=c11 -Wall -Wextra -Werror -fsanitize=undefined,address -fno-pie -no-pie)
includes=(-Itests/imu_stubs -IMyApp/Inc -Icmake/microros/libmicroros/microros_include)
cc "${flags[@]}" -Itests/reconnect_stubs "${includes[@]}" tests/test_reconnection.c -o "$bin_dir/reconnection"
cc "${flags[@]}" "${includes[@]}" MyApp/Src/drive_control.c tests/test_drive_feedback.c -lm -o "$bin_dir/drive"
cc "${flags[@]}" "${includes[@]}" MyApp/Src/imu.c MyApp/Src/imu_attitude.c MyApp/Src/gyro_calibration.c tests/test_imu.c -lm -o "$bin_dir/imu"
cc "${flags[@]}" -Wno-unused-parameter -Itests/transport_stubs -Itests/reconnect_stubs "${includes[@]}" tests/test_transport_reopen.c -o "$bin_dir/transport"
cc "${flags[@]}" "${includes[@]}" MyApp/Src/gyro_calibration.c tests/test_gyro_calibration.c -lm -o "$bin_dir/calibration"
cc "${flags[@]}" -Itests/battery_stubs "${includes[@]}" MyApp/Src/battery_monitor.c tests/test_battery_monitor.c -o "$bin_dir/battery"
cc "${flags[@]}" "${includes[@]}" MyApp/Src/drive_control.c tests/test_position_move.c -lm -o "$bin_dir/position"
cc "${flags[@]}" -Itests/xl430_stubs -IMyApp/Inc MyApp/Src/xl430.c tests/test_xl430_position.c -o "$bin_dir/xl430"
for test in reconnection drive imu transport calibration battery position xl430; do
    ASAN_OPTIONS=detect_leaks=0 "$bin_dir/$test"
done
