"""Exercise the exporter's embedded Python with typed GDB-value stand-ins."""
import copy
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

script = Path('tools/export_lqr_trace.gdb').read_text()
code = compile(script.split('\npython\n', 1)[1].split('\nend\n', 1)[0], 'export_trace', 'exec')

class Frames(list):
    @property
    def type(self):
        return SimpleNamespace(sizeof=len(self) * 92, target=lambda: SimpleNamespace(sizeof=92))

class Debugger:
    def __init__(self, instance, source):
        self.instance, self.source, self.commands = instance, source, []

    def parse_and_eval(self, expression):
        if expression == "'main.c'::%s_trace" % self.source:
            return self.instance
        if expression == "'lqr_app.c'::status.mode" or expression.startswith(('(*', '*')):
            return 0
        raise AssertionError(expression)

    def execute(self, command):
        self.commands.append(command)

with tempfile.TemporaryDirectory() as directory:
    for source, source_id, fmt, size in [('lqr', 1, 2, 68), ('lqr', 1, 3, 72), ('imu', 2, 1, 24)]:
        status = dict(mode=2, source_id=source_id, format_id=fmt, payload_size=size,
                      count=3, write_index=1, session=4, freeze_reason=4,
                      total_written=8, duplicate_frames=2)
        data = dict(input_timestamp_us=100, input_sequence=7, dt_us=5000, age_us=0,
                    error=[1]*6, torque_nm=[2, 3], duty=[.1, .2], relative_rpm=[4, 5],
                    pitch_rate_scale=.5, yaw_rate_scale=.5, accel=[1, 2, 3], gyro=[4, 5, 6])
        def frame(seq, valid):
            return dict(header=dict(timestamp_us=seq*5000, sequence=seq,
                                    business_mode=1, faults=0, event=0 if valid else 4,
                                    valid=valid), data=data)
        instance = dict(recorder=dict(status=status), frames=Frames([frame(8, 0), frame(6, 1), frame(7, 1)]))
        debugger = Debugger(instance, source)
        output = Path(directory)/f'{source}_{fmt}.json'
        with patch.dict(os.environ, TRACE_SOURCE=source, LQR_TRACE_OUT=str(output)):
            exec(code, dict(gdb=debugger))
            result = json.loads(output.read_text())
            assert result['schema_version'] == 3 and result['capacity'] == 3
            assert [f['sequence'] for f in result['frames']] == [6, 7, 8]
            assert 'error' not in result['frames'][-1] and 'accel_g' not in result['frames'][-1]
            if source == 'lqr':
                assert result['frames'][0]['relative_rpm'] == [4, 5]
                assert result['frames'][0]['yaw_rate_scale'] == (.5 if fmt==3 else 1.0)
            else:
                assert result['frames'][0]['gyro_dps'] == [4, 5, 6]
            before = output.read_bytes()
            try:
                exec(code, dict(gdb=debugger))
                raise AssertionError('Should refuse overwrite')
            except FileExistsError:
                pass
            assert output.read_bytes() == before
            assert debugger.commands[-2:] == ['monitor resume', 'detach']
        instance = copy.deepcopy(instance)
        instance['recorder']['status']['mode'] = 1
        with patch.dict(os.environ, TRACE_SOURCE=source, LQR_TRACE_OUT=str(Path(directory)/'live.json')):
            try:
                exec(code, dict(gdb=Debugger(instance, source)))
                raise AssertionError('Should refuse live trace')
            except RuntimeError as error:
                assert 'FROZEN' in str(error)
assert script.endswith('monitor resume\ndetach\n')
print('PASS: typed LQR/IMU export, chronological order, event omission, frozen-only and no overwrite')
