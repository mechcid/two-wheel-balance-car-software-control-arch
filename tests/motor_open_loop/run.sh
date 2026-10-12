#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
test_dir=$(mktemp -d /tmp/balance-car-motor-test.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  -Itests/motor_open_loop/stubs -Imodule/base/inc \
  -c module/base/src/encoders.c -o "$test_dir/encoders.o"
c++ -std=c++17 -Wall -Wextra -Werror -fsanitize=undefined \
  -Itests/motor_open_loop/stubs -Imodule/base/inc -Imodule/overlay_app/inc \
  tests/motor_open_loop/test.cpp module/overlay_app/src/motor_app.cpp \
  "$test_dir/encoders.o" -o "$test_dir/test"
"$test_dir/test"
