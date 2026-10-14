#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
test_dir=$(mktemp -d /tmp/balance-car-state-test.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  -Itests/motor_open_loop/stubs -Imodule/base/inc -Imodule/overlay_app/inc \
  -include tests/status_state/hal_extra.h \
  tests/status_state/test.c module/overlay_app/src/odometry_app.c \
  module/overlay_app/src/status_state_app.c -lm -o "$test_dir/test"
"$test_dir/test"
