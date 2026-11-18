"""Deterministic ground-truth plant for exercising the complete ROS pipeline."""
import math
import time
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
class Mock(Node):
    def __init__(self):
        super().__init__('mock_chassis')
        self.pose=self.create_publisher(PoseStamped,'/transformed/pose',100)
        self.imu=self.create_publisher(Imu,'/imu/data',100)
        self.ready=self.create_publisher(Bool,'/prerobot/bridge_ready',10)
        self.sent=self.create_publisher(TwistStamped,'/prerobot/sent_cmd',100)
        self.create_subscription(TwistStamped,'/prerobot/cmd_vel',self.cmd,1)
        self.v=[0.,0.]; self.p=[0.,0.]; self.target=[0.,0.]; self.last=time.monotonic(); self.received=0.
        self.create_timer(.01,self.tick)
    def cmd(self,m):
        self.target=[m.twist.linear.x,m.twist.linear.y]; self.received=time.monotonic()
        self.sent.publish(m)
    def tick(self):
        now=time.monotonic(); dt=now-self.last; self.last=now
        if now-self.received>.05: self.target=[0.,0.]
        a=[max(-1.5,min(1.5,(t-v)/.08)) for t,v in zip(self.target,self.v)]
        for i in range(2):
            self.p[i]+=self.v[i]*dt+.5*a[i]*dt*dt; self.v[i]+=a[i]*dt
        p=PoseStamped(); p.header.stamp=self.get_clock().now().to_msg(); p.header.frame_id='map'
        p.pose.position.x,p.pose.position.y=self.p; p.pose.orientation.w=1.; self.pose.publish(p)
        m=Imu(); m.header.stamp=p.header.stamp; m.header.frame_id='imu_link'; m.orientation.w=1.
        m.linear_acceleration.x,m.linear_acceleration.y=a; m.linear_acceleration.z=9.80665
        self.imu.publish(m); self.ready.publish(Bool(data=True))
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO); node=Mock()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node(); rclpy.shutdown()
