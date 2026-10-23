#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
test_dir=$(mktemp -d /tmp/balance-car-lqr-test.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  -Itests/motor_open_loop/stubs -Imodule/base/inc -Imodule/overlay_app/inc \
  -include tests/status_state/hal_extra.h tests/lqr/test.c module/overlay_app/src/lqr_app.c module/overlay_app/src/trace_app.c module/overlay_app/src/debug_app.c module/overlay_app/src/trace_sources.c -lm -o "$test_dir/test"
"$test_dir/test"

"$test_dir/test" imu
