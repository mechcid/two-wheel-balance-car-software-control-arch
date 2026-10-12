#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
binary=$(mktemp /tmp/balance-car-imu-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  -Itests/imu_timing/stubs -Imodule/base/inc -Imodule/overlay_app/inc \
  tests/imu_timing/test.c module/base/src/timebase.c module/overlay_app/src/sensor_app.c \
  module/overlay_app/src/motion_app.c module/overlay_app/src/motion_estimator.c -lm -o "$binary"
"$binary"
