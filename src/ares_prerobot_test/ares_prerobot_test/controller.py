"""Single-owner chassis controller with a local HTTP API."""
import json
import math
import secrets
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from .motion import Motion


class Controller(Node):
    def __init__(self):
        super().__init__('chassis_controller')
        self.declare_parameter('http_host', '127.0.0.1')
        self.declare_parameter('http_port', 8766)
        self.declare_parameter('angular_accel', 2.0)
        self.declare_parameter('angular_jerk', 8.0)
        self.declare_parameter('lease_seconds', 1.5)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('max_linear_accel', 4.0)
        self.declare_parameter('max_linear_jerk', 20.0)
        self.lease_seconds = float(self.get_parameter('lease_seconds').value)
        self.limits = {
            'speed': float(self.get_parameter('max_linear_speed').value),
            'wz': float(self.get_parameter('max_angular_speed').value),
            'amax': float(self.get_parameter('max_linear_accel').value),
            'jerkmax': float(self.get_parameter('max_linear_jerk').value),
        }
        self.motion = Motion(
            float(self.get_parameter('angular_accel').value),
            float(self.get_parameter('angular_jerk').value))
        self.lock = threading.RLock()
        self.owner = None
        self.token = None
        self.lease_until = 0.0
        self.mode = 'idle'
        self.active = False
        self.zero_frames = 0
        self.bridge_ready = False
        self.sent = [0.0, 0.0, 0.0]
        self.events = deque(maxlen=20)
        self.last_tick = time.monotonic()
        self.pub = self.create_publisher(TwistStamped, '/prerobot/cmd_vel', 1)
        self.create_subscription(Bool, '/prerobot/bridge_ready', self.on_bridge, 10)
        self.create_subscription(TwistStamped, '/prerobot/sent_cmd', self.on_sent, 10)
        self.create_timer(0.01, self.tick)
        host = str(self.get_parameter('http_host').value)
        port = int(self.get_parameter('http_port').value)
        self.server = ThreadingHTTPServer((host, port), make_handler(self))
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.event('controller ready')
        self.get_logger().info(f'CLI/Web API ready at http://{host}:{port}')

    def event(self, message):
        self.events.append({'time': time.strftime('%H:%M:%S'), 'text': message})

    def on_bridge(self, message):
        with self.lock:
            if self.bridge_ready and not message.data and self.active:
                self.active = False
                self.mode = 'usb_lost'
                self.event('USB lost; command transmission stopped')
            self.bridge_ready = message.data

    def on_sent(self, message):
        with self.lock:
            self.sent = [message.twist.linear.x, message.twist.linear.y,
                         message.twist.angular.z]

    def check_owner(self, token):
        if not self.token or token != self.token or time.monotonic() >= self.lease_until:
            raise ValueError('control lease expired or token invalid')

    def request(self, action, data):
        with self.lock:
            now = time.monotonic()
            if action == 'claim':
                if self.owner and now < self.lease_until:
                    raise ValueError(f'control held by {self.owner}')
                self.owner = str(data.get('client', 'operator'))[:40]
                self.token = secrets.token_urlsafe(24)
                self.lease_until = now + self.lease_seconds
                self.active = False
                self.motion.halt()
                self.mode = 'idle'
                self.event(f'control claimed by {self.owner}')
                return {'token': self.token, 'lease_seconds': self.lease_seconds}
            self.check_owner(data.get('token'))
            if action == 'heartbeat':
                self.lease_until = now + self.lease_seconds
            elif action == 'command':
                values = [float(data[key]) for key in ('vx', 'vy', 'wz', 'amax', 'jerkmax')]
                vx, vy, wz, amax, jerkmax = values
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('all values must be finite')
                if math.hypot(vx, vy) > self.limits['speed'] or abs(wz) > self.limits['wz']:
                    raise ValueError('target speed exceeds configured limit')
                if not 0 < amax <= self.limits['amax'] or not 0 < jerkmax <= self.limits['jerkmax']:
                    raise ValueError('amax or jerkmax outside configured limit')
                if not self.bridge_ready:
                    raise ValueError('USB bridge not ready')
                self.motion.set_target(*values)
                self.active = True
                self.zero_frames = 0
                self.mode = 'tracking'
                self.lease_until = now + self.lease_seconds
                self.event(f'target {vx:.3f} {vy:.3f} {wz:.3f}')
            elif action == 'stop':
                if not self.bridge_ready:
                    raise ValueError('USB bridge not ready')
                self.motion.set_target(0.0, 0.0, 0.0, self.motion.amax, self.motion.jerkmax)
                self.active = self.bridge_ready
                self.zero_frames = 5
                self.mode = 'stopping'
                self.event('operator stop')
            elif action == 'halt':
                if not self.bridge_ready:
                    raise ValueError('USB bridge not ready')
                self.motion.halt()
                self.active = self.bridge_ready
                self.zero_frames = 5
                self.mode = 'halting'
                self.event('operator halt')
            elif action == 'release':
                self.active = False
                self.owner = None
                self.token = None
                self.mode = 'idle'
                self.event('control released; transmission stopped')
            else:
                raise ValueError('unknown action')
            return self.snapshot()

    def snapshot(self):
        now = time.monotonic()
        return {
            'owner': self.owner if now < self.lease_until else None,
            'lease_remaining': max(0.0, self.lease_until - now) if self.owner else 0.0,
            'mode': self.mode,
            'usb_ready': self.bridge_ready,
            'transmitting': self.active and self.bridge_ready,
            'target': self.motion.target[:],
            'command': self.motion.v[:],
            'sent': self.sent[:],
            'limits': self.limits,
            'events': list(self.events),
        }

    def tick(self):
        with self.lock:
            now = time.monotonic()
            dt = min(0.05, now - self.last_tick)
            self.last_tick = now
            if self.owner and now >= self.lease_until:
                self.owner = None
                self.token = None
                self.active = False
                self.mode = 'control_lost'
                self.event('control lease lost; transmission stopped')
            if not self.active or not self.bridge_ready:
                return
            if self.mode == 'halting':
                self.zero_frames -= 1
            else:
                self.motion.step(dt)
                if self.mode == 'stopping' and self.motion.at_target():
                    self.zero_frames -= 1
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'command'
            message.twist.linear.x, message.twist.linear.y, message.twist.angular.z = self.motion.v
            self.pub.publish(message)
            if self.mode in ('stopping', 'halting') and self.zero_frames <= 0:
                self.active = False
                self.mode = 'idle'
                self.event('explicit stop complete')

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join()


def make_handler(controller):
    web = Path(get_package_share_directory('ares_prerobot_test')) / 'web'

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, code, body):
            raw = json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path == '/api/status':
                with controller.lock:
                    self.respond(200, controller.snapshot())
                return
            files = {'/': ('index.html', 'text/html'),
                     '/app.css': ('app.css', 'text/css'),
                     '/app.js': ('app.js', 'application/javascript')}
            if self.path not in files:
                self.send_error(404)
                return
            filename, content_type = files[self.path]
            raw = (web / filename).read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', content_type + '; charset=utf-8')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_POST(self):
            if self.path not in ('/api/claim', '/api/heartbeat', '/api/command',
                                 '/api/stop', '/api/halt', '/api/release'):
                self.send_error(404)
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if length > 4096:
                    raise ValueError('request too large')
                data = json.loads(self.rfile.read(length))
                result = controller.request(self.path.rsplit('/', 1)[-1], data)
                self.respond(200, result)
            except (ValueError, KeyError, TypeError) as exc:
                self.respond(400, {'error': str(exc)})

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
