"""Offline tests only: never connect to a debug probe."""
import copy
import csv
import importlib.util
import json
import math
from pathlib import Path
import runpy
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from trace_report import flatten, report


def sample(i):
    return dict(timestamp_us=(0xffff0000+i*5000)&0xffffffff, sequence=i,
        business_mode=1, faults=0, event=0, valid=1,
        input_timestamp_us=(0xffff0000+i*5000-200)&0xffffffff, input_sequence=i+100,
        dt_us=5000, age_us=200, pitch_rate_scale=.5,
        error=[.001*i, .01, .03*math.sin(i/3), .1, .02, .5*math.cos(i/3)],
        duty=[.2*math.sin(i/3), .18*math.sin(i/3)],
        torque_nm=[.03*math.sin(i/3), .025*math.sin(i/3)], relative_rpm=[i, -i])


def fixture():
    frames = [sample(i) for i in range(63)]
    event = sample(63)
    event.update(event=1, valid=0, business_mode=0)
    frames.append(event)
    return dict(schema_version=3, capacity=64, nominal_sample_hz=200,
                status=dict(source_id=1, format_id=2, payload_size=68, freeze_reason=1), frames=frames)


with tempfile.TemporaryDirectory(prefix='trace-tools-test-') as temporary:
    root = Path(temporary)
    document = fixture()
    source, fields, rows = flatten(document)
    assert rows[-1]['time_s'] == .315 and 'pitch_rad' not in rows[-1]
    assert rows[14]['time_s'] == .07  # Counter wrapped; relative time is continuous.
    path = root/'synthetic_lqr.json'
    path.write_text(json.dumps(document))
    outputs = report(path)
    assert outputs[-1].read_bytes().startswith(b'\x89PNG')
    with outputs[0].open() as stream:
        csv_rows = list(csv.DictReader(stream))
    assert len(csv_rows) == 64 and csv_rows[-1]['pitch_rad'] == ''
    summary = json.loads(outputs[1].read_text())
    assert summary['valid_samples'] == 63 and summary['window_s'] == .315
    assert summary['sample_interval_us'] == [5000]*62
    try:
        report(path)
        raise AssertionError('overwrite accepted')
    except FileExistsError:
        pass
    assert summary['tuning']['yaw_rate_scale'] == [1.0]
    new_document = copy.deepcopy(document)
    new_document['status'].update(format_id=3, payload_size=72)
    for f in new_document['frames']:
        if f['valid']: f['yaw_rate_scale']=.5
    _, _, new_rows=flatten(new_document)
    assert new_rows[0]['yaw_rate_scale']==.5 and 'yaw_rate_scale' not in new_rows[-1]
    event_only = copy.deepcopy(document)
    event_only['frames'] = [document['frames'][-1]]
    event_path = root/'event_only.json'
    event_path.write_text(json.dumps(event_only))
    report(event_path)
    bad = copy.deepcopy(document)
    bad['schema_version'] = 1
    try:
        flatten(bad)
        raise AssertionError('old schema accepted')
    except ValueError:
        pass
    imu = copy.deepcopy(document)
    imu['status'] = dict(source_id=2, format_id=1, payload_size=24, freeze_reason=4)
    imu['frames'] = [dict(timestamp_us=5000, sequence=1, business_mode=1, faults=0,
                          event=0, valid=1, accel_g=[0,0,1], gyro_dps=[1,2,3])]
    imu_path = root/'synthetic_imu.json'
    imu_path.write_text(json.dumps(imu))
    report(imu_path)

    # Exercise the actual GDB collector against typed value stand-ins.
    class Frames(list):
        @property
        def type(self):
            return SimpleNamespace(sizeof=len(self)*92, target=lambda: SimpleNamespace(sizeof=92))
    class Command:
        def __init__(self, *args): pass
    frames = Frames([dict(header={k: f[k] for k in ('timestamp_us','sequence','business_mode','faults','event','valid')},
                          data=f) for f in document['frames']])
    # Full ring's oldest slot is 2; newest terminal event is at slot 1.
    frames[:] = frames[-2:] + frames[:-2]
    status = dict(document['status'], mode=2, count=64, write_index=2,
                  session=1, total_written=200, duplicate_frames=0)
    def evaluate(expression):
        if expression == "'main.c'::lqr_trace":
            return dict(recorder=dict(status=status), frames=frames)
        if expression == "'lqr_app.c'::status.mode": return 0
        if expression.startswith(('(*','*')): return 0
        raise AssertionError(expression)
    fake = SimpleNamespace(Command=Command, COMMAND_USER=0, GdbError=RuntimeError,
        selected_thread=lambda: SimpleNamespace(is_stopped=lambda: True),
        parse_and_eval=evaluate, current_progspace=lambda: SimpleNamespace(filename=str(path)), write=lambda s: None)
    with patch.dict(sys.modules, gdb=fake):
        module = runpy.run_path(str(ROOT/'tools/trace_gdb.py'))
        captured = module['collect_trace']('lqr')
        assert [f['sequence'] for f in captured['frames']] == list(range(64))
        assert 'error' not in captured['frames'][-1]
        saved = module['save_trace']('lqr', root/'capture', make_report=False)
        assert len(json.loads(saved.read_text())['capture']['elf_sha256']) == 64
        status.update(format_id=3,payload_size=72)
        for f in frames: f['data']['yaw_rate_scale']=.5
        assert module['collect_trace']('lqr')['frames'][0]['yaw_rate_scale']==.5
        status['mode'] = 1
        try:
            module['collect_trace']('lqr')
            raise AssertionError('live trace accepted')
        except RuntimeError as error:
            assert 'FROZEN' in str(error)
        status['mode'] = 2
        original = fake.parse_and_eval
        fake.parse_and_eval = lambda e: 1 if e == "'lqr_app.c'::status.mode" else original(e)
        try:
            module['collect_trace']('lqr')
            raise AssertionError('active LQR accepted')
        except RuntimeError as error:
            assert 'active' in str(error)
print('PASS: JSON/CSV/PNG, wraparound, events, overwrite protection, typed ring order and capture guards (offline)')
