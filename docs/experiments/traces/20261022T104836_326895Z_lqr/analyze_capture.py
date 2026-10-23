"""Offline diagnosis of this capture; K read from the capture-matching ELF via GDB.
Does not connect to hardware or change controller settings.
"""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
capture=json.loads((ROOT/'lqr_trace.json').read_text())
frames=[f for f in capture['frames'] if f['valid'] and f['event']==0]
k=np.array([[-.442112088,-.221407965,-1.00458324,-.403306544,-.141159505,-.176613986],
            [-.442081660,.221433103,-1.00463545,-.403315753,.141176447,-.176625505]])
e=np.array([f['error'] for f in frames]); torque=np.array([f['torque_nm'] for f in frames])
duty=np.array([f['duty'] for f in frames]);rpm=np.array([f['relative_rpm'] for f in frames])
t=(np.array([f['timestamp_us'] for f in frames],dtype=np.int64)-frames[0]['timestamp_us'])/1e6
terms=-e[:,None,:]*k[None,:,:]
terms[:,:,5]*=np.array([f['pitch_rate_scale'] for f in frames])[:,None]
raw=(torque+.01139*rpm*np.pi/30)/(.03677*12)
residual=float(np.max(abs(terms.sum(2)-torque)))
assert residual<1e-6, 'K does not reconstruct the captured controller output'
rms=lambda a:np.sqrt(np.mean(a*a,axis=0))
common=torque.mean(1);diff=(torque[:,0]-torque[:,1])/2
wheel_yaw=(rpm[:,1]-rpm[:,0])*np.pi/30*.0325/.185
lags=[]
for lag in range(-8,9):
 x=wheel_yaw[max(lag,0):len(t)+min(lag,0)]
 y=e[max(-lag,0):len(t)-max(lag,0),4]
 lags.append(dict(wheel_index_minus_gyro_index=lag,correlation=float(np.corrcoef(x,y)[0,1])))
peaks=[i for i in range(1,len(t)-1) if e[i,4]>e[i-1,4] and e[i,4]>=e[i+1,4] and e[i,4]>1]
metrics=dict(valid_frames=len(frames),stop_event=capture['frames'][-1],
    interval_us_minmax=[int(np.diff(np.array([f['timestamp_us'] for f in frames])).min()),int(np.diff(np.array([f['timestamp_us'] for f in frames])).max())],
    input_interval_us_minmax=[min(f['dt_us'] for f in frames),max(f['dt_us'] for f in frames)],
    input_age_us_minmax=[min(f['age_us'] for f in frames),max(f['age_us'] for f in frames)],
    consecutive_output_sequences=bool(np.all(np.diff([f['sequence'] for f in frames])==1)),
    consecutive_input_sequences=bool(np.all(np.diff([f['input_sequence'] for f in frames])==1)),
    torque_replay_max_error_nm=residual,duty_replay_max_error=float(np.max(abs(np.clip(raw,-.4,.4)-duty))),
    saturated_wheel_frames=np.sum(abs(duty)>=.39999,axis=0).tolist(),
    any_wheel_saturated_frames=int(np.any(abs(duty)>=.39999,axis=1).sum()),
    opposite_duty_frames=int(np.sum(duty[:,0]*duty[:,1]<0)),
    torque_term_order=['position','yaw_angle','pitch','forward_speed','yaw_rate','pitch_rate'],
    torque_terms_rms_nm=rms(terms).tolist(),
    common_torque_rms_nm=float(rms(common)),differential_torque_rms_nm=float(rms(diff)),
    pitch_deg_minmax=np.rad2deg([e[:,2].min(),e[:,2].max()]).tolist(),
    yaw_deg_minmax=np.rad2deg([e[:,1].min(),e[:,1].max()]).tolist(),
    yaw_rate_deg_s_minmax=np.rad2deg([e[:,4].min(),e[:,4].max()]).tolist(),
    position_m_minmax=[float(e[:,0].min()),float(e[:,0].max())],
    yaw_positive_peak_times_s=t[peaks].tolist(),
    yaw_peak_period_s=np.diff(t[peaks]).tolist(),
    wheel_yaw_assumptions=dict(radius_m=.0325,track_width_m=.185,no_slip=True),
    wheel_gyro_lag_correlations=lags,
    limitations='0.31 s window; lag correlation does not identify sensor latency or prove causality; model torque is not measured torque')
(ROOT/'analysis_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
fig,axs=plt.subplots(4,1,figsize=(11,12),layout='constrained')
labels=['Position','Yaw angle','Pitch','Speed','Yaw rate','Pitch rate (x0.5)']
axs[0].bar(labels,rms(terms)[0,:],color=['#729bb8']*4+['#c74c48','#729bb8'])
axs[0].set_ylabel('Left torque term RMS (Nm)')
axs[0].set_title('Yaw-rate feedback is the largest individual torque contribution')
axs[1].plot(t,common,label='Common torque: (L+R)/2')
axs[1].plot(t,diff,label='Differential torque: (L-R)/2')
axs[1].set_ylabel('Requested torque (Nm)');axs[1].legend()
axs[2].plot(t,e[:,4],label='IMU-derived yaw rate')
axs[2].plot(t,wheel_yaw,label='Wheel-derived yaw rate (no-slip model)')
axs[2].set_ylabel('Yaw rate (rad/s)');axs[2].legend()
axs[2].set_title('Phase discrepancy: not a direct measurement of sensor latency')
axs[3].plot(t,100*raw[:,0],label='Left before clamp',linestyle='--')
axs[3].plot(t,100*duty[:,0],label='Left applied')
axs[3].plot(t,100*raw[:,1],label='Right before clamp',linestyle='--')
axs[3].plot(t,100*duty[:,1],label='Right applied')
axs[3].axhline(40,color='gray',linestyle=':');axs[3].axhline(-40,color='gray',linestyle=':')
axs[3].set_ylabel('Duty (%)');axs[3].legend(ncol=2)
for ax in axs:
 ax.grid(alpha=.25)
for ax in axs[1:]:ax.set_xlabel('Time since oldest retained sample (s)')
fig.suptitle('2026-10-22 LQR capture | last 0.31 s | observed, not a root-cause proof')
fig.savefig(ROOT/'analysis_diagnosis.png',dpi=150)
plt.close(fig)
print(json.dumps(metrics,indent=2))
