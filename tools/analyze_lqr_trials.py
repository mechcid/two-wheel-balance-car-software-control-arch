import json,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]/'docs/experiments'
summary=[]
for index in (1,2):
    path=root/f'lqr_trial_{index:02d}.json'
    trial=json.loads(path.read_text());frames=trial['frames']
    rows=[r for r in frames if r['mode']==1 and r['dt_us']>0]
    t=np.array([(r['timestamp_us']-frames[0]['timestamp_us'])/1e6 for r in rows])
    e=np.array([r['error'] for r in rows]);d=np.array([r['duty'] for r in rows]);tau=np.array([r['torque_nm'] for r in rows])
    with path.with_suffix('.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['time_s','dt_us','imu_age_us','x_error_m','yaw_error_rad','pitch_rad','v_error_mps','yaw_rate_radps','pitch_rate_radps','tau_L_Nm','tau_R_Nm','duty_L','duty_R','rpm_L','rpm_R'])
        for stamp,r in zip(t,rows): w.writerow([stamp,r['dt_us'],r['age_us'],*r['error'],*r['torque_nm'],*r['duty'],*r.get('relative_rpm',['',''])])
    fig,axs=plt.subplots(4,1,figsize=(9,9),sharex=True)
    axs[0].plot(t,np.rad2deg(e[:,2]),label='Pitch');axs[0].plot(t,np.rad2deg(e[:,1]),label='Yaw error');axs[0].set_ylabel('Angle (deg)')
    axs[1].plot(t,100*e[:,0],label='Forward position error');axs[1].set_ylabel('Position (cm)')
    for i,label in enumerate(['Left','Right']):axs[2].plot(t,100*d[:,i],label=label)
    axs[2].set_ylabel('Duty (%)')
    if 'relative_rpm' in rows[0]:
        rpm=np.array([r['relative_rpm'] for r in rows])
        for i,label in enumerate(['Left','Right']):axs[3].plot(t,rpm[:,i],label=label)
        axs[3].set_ylabel('Encoder speed (rpm)')
    else:
        for i,label in enumerate(['Left','Right']):axs[3].plot(t,tau[:,i],label=label)
        axs[3].set_ylabel('Requested torque (Nm)')
    for ax in axs:ax.grid(alpha=.3);ax.legend(loc='best')
    axs[-1].set_xlabel('Time since trial start (s)')
    fig.suptitle(f'Trial {index}: hand-supported, 2 s, 40 Hz trace / 200 Hz control')
    fig.tight_layout();fig.savefig(path.with_suffix('.png'),dpi=160);plt.close(fig)
    summary.append(dict(trial=index,recorded_active_frames=len(rows),sampled_dt_us=[min(r['dt_us'] for r in rows),max(r['dt_us'] for r in rows)],max_sampled_imu_age_us=max(r['age_us'] for r in rows),pitch_deg=[float(np.rad2deg(e[:,2]).min()),float(np.rad2deg(e[:,2]).max())],peak_abs_duty=np.abs(d).max(axis=0).tolist(),sampled_saturated_frames=int(np.any(np.abs(d)>=.39999,axis=1).sum())))
(root/'lqr_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
