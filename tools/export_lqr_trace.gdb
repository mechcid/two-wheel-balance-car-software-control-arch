# PRECONDITION: stop motors through the application before attaching GDB.
# Attaching itself halts the CPU; the checks below cannot make a running attach safe.
set pagination off
target remote localhost:3333
python
import json,os
from pathlib import Path
try:
    if int(gdb.parse_and_eval("'lqr_app.c'::status.mode")) == 1:
        raise RuntimeError('LQR active: stop motors BEFORE attaching GDB')
    if any(int(gdb.parse_and_eval(expr)) for expr in [
        '(*(unsigned int*)0x4001080c) & 0x8000',
        '*(unsigned int*)0x4000043c','*(unsigned int*)0x40000440']):
        raise RuntimeError('Motor outputs are not disabled')
    source=os.environ.get('TRACE_SOURCE','lqr').lower()
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
    path=Path(os.environ.get('LQR_TRACE_OUT','docs/experiments/%s_trace.json'%source))
    with path.open('x') as out:json.dump(dict(schema_version=3,capacity=capacity,
        nominal_sample_hz=200,status=meta,frames=rows),out,indent=2)
    print('Saved %d frozen frames to %s'%(n,path))
except Exception:
    # Avoid leaving a halted MCU behind on validation or file errors.
    gdb.execute('monitor resume')
    gdb.execute('detach')
    raise
end
monitor resume
detach
