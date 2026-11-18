"""Reproducible offline analysis; derivatives use sensor acquisition stamps."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation


def load(path):
    with path.open() as f: return list(csv.DictReader(f))


def differentiate(t, xy, window_seconds=.21, max_gap=.15):
    t=np.asarray(t,dtype=float); xy=np.asarray(xy,dtype=float)
    if len(t)<9 or not np.isfinite(t).all() or not np.isfinite(xy).all():
        raise ValueError('Need at least 9 finite samples')
    if np.any(np.diff(t)<=0): raise ValueError('Sensor timestamps must strictly increase')
    if np.max(np.diff(t))>max_gap: raise ValueError('Pose gap exceeds analysis max_gap; do not differentiate across missing data')
    dt=float(np.median(np.diff(t)))
    grid=np.arange(t[0],t[-1],dt)
    n=max(9,int(round(window_seconds/dt))|1)
    if len(grid)<2*n: raise ValueError('Recording too short for requested differentiation window')
    points=CubicSpline(t,xy,axis=0)(grid)
    derivatives=[savgol_filter(points,n,4,deriv=d,delta=dt,axis=0) for d in (1,2,3)]
    # Discard boundary derivatives rather than include extrapolation artifacts in maxima.
    valid=slice(n//2,-(n//2))
    return grid[valid],*[d[valid] for d in derivatives],dict(dt=dt,window_samples=n,window_seconds=n*dt,polynomial_order=4,max_gap=float(np.max(np.diff(t))))


def imu_metrics(rows, gravity, frame, window=.21, max_gap=.15):
    rows=[r for r in rows if int(r['orientation_available']) and r['frame']==frame]
    if len(rows)<9: return dict(available=False,reason='No usable orientation in requested IMU frame')
    t=np.array([float(r['stamp']) for r in rows]); q=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows])
    a=np.array([[float(r[k]) for k in ('ax','ay','az')] for r in rows])
    if not np.isfinite(q).all() or not np.isfinite(a).all() or np.any(np.linalg.norm(q,axis=1)<.99):
        return dict(available=False,reason='Invalid IMU quaternion or acceleration')
    world=Rotation.from_quat(q).apply(a)-[0,0,gravity]
    # First derivative of world acceleration, using the same fitting method.
    tg, derivative, _, _, settings=differentiate(t,world,window,max_gap)
    dt=settings['dt']; n=settings['window_samples']
    grid=np.arange(t[0],t[-1],dt)
    uniform=np.column_stack([np.interp(grid,t,world[:,i]) for i in range(3)])
    smooth=savgol_filter(uniform,n,4,axis=0)[n//2:-(n//2)]
    return dict(available=True,peak_horizontal_acceleration=float(np.linalg.norm(smooth[:,:2],axis=1).max()),
                peak_horizontal_jerk=float(np.linalg.norm(derivative[:,:2],axis=1).max()),filter=settings,
                convention='q rotates sensor frame into ENU world; raw acceleration includes +g at rest')


def analyze(directory, window=.21, max_gap=.15, stable_seconds=.5, plateau_tolerance=.03, imu_frame='imu_link'):
    directory=Path(directory); meta=json.loads((directory/'metadata.json').read_text())
    rows=load(directory/'pose.csv')
    t,v,a,j,settings=differentiate([float(r['stamp']) for r in rows],[[float(r['x']),float(r['y'])] for r in rows],window,max_gap)
    speed=np.linalg.norm(v,axis=1); accel=np.linalg.norm(a,axis=1); jerk=np.linalg.norm(j,axis=1)
    direction=np.array([math.cos(math.radians(meta['direction_deg'])),math.sin(math.radians(meta['direction_deg']))])
    along_a=a@direction
    events=load(directory/'events.csv'); commands=load(directory/'commands.csv')
    epoch=float(commands[0]['stamp'])-float(commands[0]['elapsed'])
    hold=[float(r['elapsed'])+epoch for r in events if r['state']=='hold']
    plateau=None
    if hold:
        # Restrict platform measurement to hold phase; require a complete low-spread window.
        mask=(t>=min(hold)+window/2)&(t<=max(hold)-window/2)
        h=speed[mask]; ht=t[mask]; count=max(3,int(math.ceil(stable_seconds/settings['dt'])))
        candidates=[]
        for i in range(len(h)-count+1):
            segment=h[i:i+count]
            if np.ptp(segment)<=plateau_tolerance and np.max(np.abs(along_a[mask][i:i+count]))<=.15:
                candidates.append((float(np.mean(segment)),float(ht[i]),float(ht[i+count-1])))
        if candidates:
            value,begin,end=max(candidates)
            plateau=dict(speed=value,start_stamp=begin,end_stamp=end,target_error=value-meta['speed'])
    outcome=json.loads((directory/'outcome.json').read_text()) if (directory/'outcome.json').exists() else {'reason':'incomplete'}
    report=dict(mode=meta['mode'],outcome=outcome,filter=settings,
        peak_speed=float(speed.max()),stable_speed=plateau,
        speed_interpretation='Observed stable platform in this run; physical maximum requires a sweep' if plateau else 'No stable platform; observed speed only',
        peak_acceleration=float(accel.max()),peak_forward_acceleration=float(along_a.max()),
        peak_braking_deceleration=float(max(0.,-along_a.min())),peak_jerk=float(jerk.max()),
        units=dict(speed='m/s',acceleration='m/s^2',jerk='m/s^3'),
        sampling=dict(samples=len(rows),frame=meta['pose_frame']),
        acceleration_peak_duration_above_90_percent=float(np.sum(accel>=.9*accel.max())*settings['dt']),
        firmware=dict(profile=meta['firmware_profile'],revision=meta['firmware_revision'],acceleration_limit=meta['firmware_accel_limit']),
        imu=dict(available=False,reason='No IMU recording'))
    if (directory/'imu.csv').exists():
        try: report['imu']=imu_metrics(load(directory/'imu.csv'),9.80665,imu_frame,window,max_gap)
        except ValueError as e: report['imu']=dict(available=False,reason=str(e))
    (directory/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    with (directory/'derived.csv').open('w',newline='') as f:
        w=csv.writer(f); w.writerow(['stamp','vx','vy','ax','ay','jx','jy','speed','acceleration','jerk'])
        w.writerows(np.column_stack([t,v,a,j,speed,accel,jerk]))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,sharex=True,figsize=(10,8))
    for ax,y,label in zip(axes,(speed,accel,jerk),('Speed (m/s)','Acceleration (m/s²)','Jerk (m/s³)')):
        ax.plot(t-epoch,y); ax.set_ylabel(label); ax.grid(True)
    axes[0].plot([float(r['elapsed']) for r in commands],[math.hypot(float(r['vx']),float(r['vy'])) for r in commands],label='command',alpha=.5)
    axes[0].legend(); axes[-1].set_xlabel('Elapsed (s)')
    fig.suptitle(meta['mode']+' / '+meta['kind']+' / '+outcome['reason']); fig.tight_layout(); fig.savefig(directory/'curves.png',dpi=160); plt.close(fig)
    return report


def main():
    p=argparse.ArgumentParser(); p.add_argument('directory'); p.add_argument('--window',type=float,default=.21)
    p.add_argument('--max-gap',type=float,default=.15); p.add_argument('--imu-frame',default='imu_link')
    args=p.parse_args()
    if args.window<=0 or args.max_gap<=0: p.error('window and max-gap must be positive')
    print(json.dumps(analyze(args.directory,args.window,args.max_gap,imu_frame=args.imu_frame),indent=2))
