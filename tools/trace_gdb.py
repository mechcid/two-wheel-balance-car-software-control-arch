# Loaded by GDB (not ordinary Python). Registering this command does not touch the target.
import gdb
import hashlib
import json
import os
from pathlib import Path
import subprocess
from datetime import datetime, timezone

TRACE_ROOT = Path(__file__).resolve().parents[1]


def collect_trace(source):
    thread = gdb.selected_thread()
    if thread is None or not thread.is_stopped():
        raise RuntimeError('Pause only AFTER stopping motors, then export the frozen trace.')
    if int(gdb.parse_and_eval("'lqr_app.c'::status.mode")) == 1:
        raise RuntimeError('LQR active: stop motors BEFORE attaching GDB')
    if any(int(gdb.parse_and_eval(expr)) for expr in [
        '(*(unsigned int*)0x4001080c) & 0x8000',
        '*(unsigned int*)0x4000043c','*(unsigned int*)0x40000440']):
        raise RuntimeError('Motor outputs are not disabled')
    if source not in ('lqr','imu'):
        raise RuntimeError('TRACE_SOURCE must be lqr or imu')
    instance=gdb.parse_and_eval("'main.c'::%s_trace"%source)
    s=instance['recorder']['status']
    if int(s['mode'])!=2:
        raise RuntimeError('Selected trace is not FROZEN; do not read a live buffer')
    schema=(int(s['source_id']),int(s['format_id']),int(s['payload_size']))
    if schema not in ([(1,2,68),(1,3,72)] if source=='lqr' else [(2,1,24)]):
        raise RuntimeError('Unsupported source/payload schema')
    a=instance['frames']
    capacity=int(a.type.sizeof//a.type.target().sizeof)
    n=int(s['count']); start=0 if n<capacity else int(s['write_index'])
    if not 0<=n<=capacity or not 0<=int(s['write_index'])<capacity:
        raise RuntimeError('Invalid ring metadata')
    rows=[]
    for i in range(n):
        f=a[(start+i)%capacity]
        r={k:int(f['header'][k]) for k in ['timestamp_us','sequence','business_mode','faults','event','valid']}
        r['mode']=r['business_mode']
        if r['valid']:
            data=f['data']
            if source=='lqr':
                for key in ('input_timestamp_us','input_sequence','dt_us','age_us'):
                    r[key]=int(data[key])
                for key,count in [('error',6),('torque_nm',2),('duty',2),('relative_rpm',2)]:
                    r[key]=[float(data[key][j]) for j in range(count)]
                r['pitch_rate_scale']=float(data['pitch_rate_scale'])
                r['yaw_rate_scale']=float(data['yaw_rate_scale']) if schema[1]==3 else 1.0
            else:
                r.update(accel_g=[float(data['accel'][j]) for j in range(3)],
                         gyro_dps=[float(data['gyro'][j]) for j in range(3)])
        # Invalid/terminal frames intentionally have no measurement fields.
        rows.append(r)
    meta={k:int(s[k]) for k in ['source_id','format_id','payload_size','session',
        'freeze_reason','count','total_written','duplicate_frames']}
    return dict(schema_version=3, capacity=capacity, nominal_sample_hz=200, status=meta, frames=rows)


def save_trace(source='lqr', output_dir=None, make_report=True):
    document = collect_trace(source)
    elf = Path(gdb.current_progspace().filename).resolve()
    document['capture'] = dict(utc=datetime.now(timezone.utc).isoformat(), elf=str(elf),
                               elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest())
    if output_dir is None:
        output_dir = TRACE_ROOT / 'docs/experiments/traces' / (
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_' + source)
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / (source + '_trace.json')
    encoded = json.dumps(document, indent=2, allow_nan=False)
    with path.open('x') as stream:
        stream.write(encoded)
    gdb.write('Raw trace saved: %s\n' % path)
    if make_report:
        env = os.environ.copy()
        env.pop('PYTHONHOME', None)
        env.pop('PYTHONPATH', None)
        result = subprocess.run([env.get('TRACE_PYTHON', '/usr/bin/python3'),
                                 str(TRACE_ROOT / 'tools/trace_report.py'), str(path)],
                                env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        gdb.write(result.stdout)
        if result.returncode:
            raise RuntimeError('JSON was saved, but report generation failed. Run trace_report.py offline.')
    return path


class SaveTraceCommand(gdb.Command):
    """trace-save [lqr|imu] [output-directory]: save frozen trace + CSV + PNG. Does not resume/reset."""
    def __init__(self):
        super().__init__('trace-save', gdb.COMMAND_USER)

    def invoke(self, argument, from_tty):
        args = gdb.string_to_argv(argument)
        if len(args)>2:
            raise gdb.GdbError('Usage: trace-save [lqr|imu] [output-directory]')
        try:
            save_trace(args[0] if args else 'lqr', args[1] if len(args)>1 else None)
        except Exception as error:
            raise gdb.GdbError(str(error)) from error


SaveTraceCommand()
