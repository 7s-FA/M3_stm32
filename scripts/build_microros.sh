#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
docker run --rm -v "$project_root:/project" \
  --env MICROROS_LIBRARY_FOLDER=cmake/microros \
  --env 'RET_CFLAGS=-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections' \
  microros/micro_ros_static_library_builder:jazzy
