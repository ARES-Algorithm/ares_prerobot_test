"""Single experiment state machine and raw-message CSV journal."""
import csv
import json
import math
from pathlib import Path
import time
from datetime import datetime
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String, Float32MultiArray
from std_srvs.srv import Trigger
from .profiles import Profile

class Runner(Node):
    def __init__(self):
        super().__init__('test_runner')
        defaults = dict(mode='mock', kind='jerk', speed=.5, acceleration=1., jerk=2., hold=2.,
            rate=100., direction_deg=0., command_yaw_deg=0., command_x_sign=1., command_y_sign=1.,
            pose_topic='/transformed/pose', imu_topic='/imu/data', pose_frame='map',
            feedback_timeout=.15, max_duration=15., max_distance=5., prealign_speed=.03,
            prealign_duration=1., stop_duration=1., stop_deceleration=1., output='results',
            firmware_profile='unconfirmed', firmware_accel_limit=15., firmware_revision='unconfirmed',
            calibration_confirmed=False, notes='')
        self.cfg={k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        c=self.cfg
        self.profile=Profile(c['kind'],c['speed'],c['acceleration'],c['jerk'],c['hold'])
        for k in ('rate','feedback_timeout','max_duration','max_distance','stop_duration','stop_deceleration'):
            if not math.isfinite(c[k]) or c[k]<=0: raise ValueError(k+' must be positive')
        for k in ('prealign_speed','prealign_duration'):
            if not math.isfinite(c[k]) or c[k]<0: raise ValueError(k+' must be nonnegative')
        if c['mode'] not in ('mock','hardware'): raise ValueError('mode must be mock or hardware')
        if any(c[k] not in (-1.,1.) for k in ('command_x_sign','command_y_sign')): raise ValueError('axis signs must be +/-1')
        if not all(math.isfinite(c[k]) for k in ('direction_deg','command_yaw_deg')): raise ValueError('angles must be finite')
        self.state='idle'; self.reason=''; self.ready=False; self.ready_at=0.; self.pose_at=0.
        self.pose=None; self.pose_stamp=None; self.start_xy=None; self.files={}; self.writers={}
        self.started=0.; self.phase_started=0.; self.last_speed=0.; self.stop_speed=0.
        self.pub=self.create_publisher(TwistStamped,'/prerobot/cmd_vel',1)
        self.status=self.create_publisher(String,'/prerobot/status',10)
        self.create_subscription(PoseStamped,c['pose_topic'],self.on_pose,100)
        self.create_subscription(Imu,c['imu_topic'],self.on_imu,100)
        self.create_subscription(Bool,'/prerobot/bridge_ready',self.on_ready,10)
        self.create_subscription(TwistStamped,'/prerobot/sent_cmd',self.on_sent,100)
        self.create_subscription(Float32MultiArray,'/prerobot/usb_rx',self.on_raw,100)
        self.create_service(Trigger,'/prerobot/start',self.start)
        self.create_service(Trigger,'/prerobot/stop',self.stop)
        self.timer=self.create_timer(1/c['rate'],self.tick)
        self.get_logger().info('Ready: call /prerobot/start; mode='+c['mode'])

    def row(self, key, header, values):
        if not self.files: return
        if key not in self.writers:
            f=(self.directory/(key+'.csv')).open('w',newline='')
            self.files[key]=f; self.writers[key]=csv.writer(f); self.writers[key].writerow(header)
        self.writers[key].writerow(values)

    @staticmethod
    def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

    def on_ready(self,m): self.ready=m.data; self.ready_at=time.monotonic()
    def on_pose(self,m):
        stamp=self.stamp(m)
        p=m.pose.position
        if m.header.frame_id!=self.cfg['pose_frame'] or not all(math.isfinite(v) for v in (p.x,p.y,p.z)): return
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if age < -.01 or age>self.cfg['feedback_timeout']: return
        if self.pose_stamp is not None and stamp<=self.pose_stamp: return
        self.pose_stamp=stamp; self.pose_at=time.monotonic(); self.pose=(p.x,p.y)
        q=m.pose.orientation
        self.row('pose',['stamp','received','frame','x','y','z','qx','qy','qz','qw'],
                 [stamp,time.monotonic(),m.header.frame_id,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    def on_imu(self,m):
        a=m.linear_acceleration; w=m.angular_velocity; q=m.orientation
        self.row('imu',['stamp','received','frame','ax','ay','az','wx','wy','wz','qx','qy','qz','qw','orientation_available'],
                 [self.stamp(m),time.monotonic(),m.header.frame_id,a.x,a.y,a.z,w.x,w.y,w.z,q.x,q.y,q.z,q.w,int(m.orientation_covariance[0]!=-1)])
    def on_sent(self,m):
        self.row('sent',['stamp','received','vx','vy','wz'],[self.stamp(m),time.monotonic(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z])
    def on_raw(self,m):
        received=time.monotonic()
        self.row('usb_rx',['received','library_data_id','payload_json'],[received,m.layout.data_offset,json.dumps(list(m.data))])
        if m.layout.data_offset==0x0131 and len(m.data)==15:
            self.row('firmware',['received','sequence','uptime_sec','uptime_usec','set_internal_x','set_internal_y','set_gyro',
                'wheel1_speed','wheel2_speed','wheel3_speed','wheel4_speed','wheel1_angle','wheel2_angle','wheel3_angle','wheel4_angle','accel_limit'],
                [received,*m.data])

    def start(self,req,res):
        now=time.monotonic(); c=self.cfg
        if self.state not in ('idle','finished'): res.message='Experiment already running'; return res
        if not self.ready or now-self.ready_at>c['feedback_timeout']: res.message='Bridge not ready'; return res
        if self.pose is None or now-self.pose_at>c['feedback_timeout']: res.message='Fresh stamped pose required'; return res
        if c['mode']=='hardware' and (not c['calibration_confirmed'] or c['firmware_profile']=='unconfirmed' or c['firmware_revision']=='unconfirmed'):
            res.message='Set firmware_profile, firmware_revision and calibration_confirmed after axis calibration'; return res
        if c['max_duration'] < c['prealign_duration']+self.profile.duration+c['stop_duration']:
            res.message='max_duration shorter than configured experiment'; return res
        self.directory=Path(c['output']).expanduser()/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True)
        self.files={'events':(self.directory/'events.csv').open('w',newline='')}
        self.writers={'events':csv.writer(self.files['events'])}
        self.writers['events'].writerow(['elapsed','state','speed','reason'])
        self.started=now; self.phase_started=now; self.start_xy=self.pose; self.reason=''
        self.state='prealign'; self.last_speed=0.
        (self.directory/'metadata.json').write_text(json.dumps(c,indent=2))
        res.success=True; res.message=str(self.directory); return res

    def stopping(self,reason):
        if self.state not in ('idle','finished','stopping'):
            self.state='stopping'; self.phase_started=time.monotonic(); self.stop_speed=self.last_speed; self.reason=reason
    def stop(self,req,res):
        self.stopping('operator_stop'); res.success=True; res.message=self.state; return res

    def publish(self,v):
        c=self.cfg; angle=math.radians(c['direction_deg']+c['command_yaw_deg'])
        m=TwistStamped(); m.header.stamp=self.get_clock().now().to_msg(); m.header.frame_id='command'
        m.twist.linear.x=v*math.cos(angle)*c['command_x_sign']; m.twist.linear.y=v*math.sin(angle)*c['command_y_sign']
        self.pub.publish(m); self.last_speed=v
        self.row('commands',['stamp','elapsed','state','vx','vy'],[self.stamp(m),time.monotonic()-self.started,self.state,m.twist.linear.x,m.twist.linear.y])

    def finish(self):
        self.publish(0.)
        (self.directory/'outcome.json').write_text(json.dumps(dict(reason=self.reason,elapsed=time.monotonic()-self.started),indent=2))
        for f in self.files.values(): f.close()
        self.files={}; self.writers={}; self.state='finished'
        self.get_logger().info('Finished '+self.reason+': '+str(self.directory))

    def tick(self):
        now=time.monotonic(); c=self.cfg
        self.status.publish(String(data=json.dumps(dict(state=self.state,reason=self.reason))))
        if self.state in ('idle','finished'): return
        fault = (not self.ready or now-self.ready_at>c['feedback_timeout'] or now-self.pose_at>c['feedback_timeout'])
        if fault:
            self.stopping('feedback_lost'); self.stop_speed=0.
        elif now-self.started>=c['max_duration']: self.stopping('duration_limit')
        elif math.dist(self.pose,self.start_xy)>=c['max_distance']: self.stopping('distance_limit')
        if self.state=='prealign':
            if now-self.phase_started>=c['prealign_duration']:
                self.state='execute'; self.phase_started=now
            else:
                self.publish(c['prealign_speed']*math.sin(math.pi*(now-self.phase_started)/c['prealign_duration'])**2); return
        if self.state=='execute':
            v,phase=self.profile.sample(now-self.phase_started)
            self.row('events',['elapsed','state','speed','reason'],[now-self.started,phase,v,''])
            if phase=='finished': self.stopping('completed')
            else: self.publish(v); return
        if self.state=='stopping':
            dt=now-self.phase_started
            v=max(0.,self.stop_speed-c['stop_deceleration']*dt)
            self.publish(v)
            if dt>=max(c['stop_duration'],self.stop_speed/c['stop_deceleration']): self.finish()

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO); node=Runner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node.files:
            node.reason='process_shutdown'; node.finish()
        node.destroy_node(); rclpy.shutdown()
