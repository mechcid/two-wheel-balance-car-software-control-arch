#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
test_dir=$(mktemp -d /tmp/balance-car-pipeline.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT
python3 - "$test_dir/pipeline.h" "${1:-Core/Src/main.c}" <<'PY'
from pathlib import Path
import sys
s=Path(sys.argv[2]).read_text()
a=s.index('    SensorApp_Poll();')
b=s.index('    DebugApp_Poll(',a)
body=s[a:b]
globals='\n'.join(line for line in s.splitlines() if line.startswith('static ') and ('last_state_deadline_us;' in line or 'last_control_sample_sequence;' in line or 'have_control_sample;' in line))
init='last_state_deadline_us=Timebase_NowUs();' if 'last_state_deadline_us' in body else ''
Path(sys.argv[1]).write_text(globals+'\nstatic void pipeline_init(void) {'+init+'}\nstatic void pipeline_iteration(void) {\n'+body+'}\n')
Path(sys.argv[1]).with_name('main.h').write_text('#include "stm32f1xx_hal.h"\n')
PY
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  -I"$test_dir" -Itests/motor_open_loop/stubs -Imodule/base/inc \
  -Imodule/pose_estimate/inc -Imodule/overlay_app/inc \
  -include tests/status_state/hal_extra.h \
  tests/control_pipeline/test.c module/pose_estimate/src/sensor_app.c \
  module/pose_estimate/src/motion_app.c module/pose_estimate/src/motion_estimator.c \
  module/overlay_app/src/status_state_app.c module/pose_estimate/src/odometry_app.c \
  module/overlay_app/src/lqr_app.c -lm -o "$test_dir/test"
for delay in 13000 10000 11000 12000 14000; do "$test_dir/test" "$delay"; done
