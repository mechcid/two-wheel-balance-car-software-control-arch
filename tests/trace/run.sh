#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
binary=$(mktemp /tmp/balance-car-trace-test.XXXXXX)
trap 'rm -f "$binary"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -Imodule/overlay_app/inc \
  tests/trace/test.c module/overlay_app/src/trace_app.c -o "$binary"
"$binary"
python3 tests/trace/export_test.py
