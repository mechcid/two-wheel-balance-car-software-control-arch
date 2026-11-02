#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
test_dir=$(mktemp -d /tmp/balance-car-motion-rate.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT
for hz in 100 200 400; do
  cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -DMOTION_UPDATE_HZ="$hz" \
    -Itests/imu_timing/stubs -Imodule/pose_estimate/inc \
    tests/motion_rate/test.c module/pose_estimate/src/motion_app.c \
    module/pose_estimate/src/motion_estimator.c -lm -o "$test_dir/test"
  "$test_dir/test"
  "$test_dir/test" streaming
done
