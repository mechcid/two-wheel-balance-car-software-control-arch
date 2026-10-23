#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
binary=$(mktemp /tmp/balance-car-oled-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -Itests/imu_timing/stubs -Imodule/base/inc \
  -include tests/oled/hal_extra.h tests/oled/test.c module/base/src/oled.c -o "$binary"
"$binary"
