"""Single-owner chassis controller with a local HTTP API."""
import json
import math
import secrets
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from .motion import Motion
from .recording import SessionBag
from .telemetry import LiveTelemetry


class Controller(Node):
    def __init__(self):
        super().__init__('chassis_controller')
        self.declare_parameter('http_host', '127.0.0.1')
        self.declare_parameter('http_port', 8766)
        self.declare_parameter('angular_accel', 0.5)
        self.declare_parameter('angular_jerk', 2.0)
        self.declare_parameter('lease_seconds', 1.5)
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 0.3)
        self.declare_parameter('max_linear_accel', 0.5)
        self.declare_parameter('max_linear_jerk', 2.0)
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('wheel_rpm_ratio', 1.0)
        self.declare_parameter('bag_directory', 'recordings')
        self.lease_seconds = float(self.get_parameter('lease_seconds').value)
        self.limits = {
            'speed': float(self.get_parameter('max_linear_speed').value),
            'wz': float(self.get_parameter('max_angular_speed').value),
            'amax': float(self.get_parameter('max_linear_accel').value),
            'jerkmax': float(self.get_parameter('max_linear_jerk').value),
        }
        if not math.isfinite(self.lease_seconds) or self.lease_seconds <= 0:
            raise ValueError('lease_seconds must be positive and finite')
        if any(not math.isfinite(value) or value <= 0 for value in self.limits.values()):
            raise ValueError('control limits must be positive and finite')
        self.motion = Motion(
            float(self.get_parameter('angular_accel').value),
            float(self.get_parameter('angular_jerk').value))
        self.telemetry = LiveTelemetry(
            radius=float(self.get_parameter('wheel_radius').value),
            rpm_ratio=float(self.get_parameter('wheel_rpm_ratio').value))
        self.bag_directory = str(self.get_parameter('bag_directory').value)
        self.lock = threading.RLock()
        self.bag = None
        self.bag_stop_at = None
        self.last_bag_path = None
        self.owner = None
        self.token = None
        self.lease_until = 0.0
        self.mode = 'idle'
        self.active = False
        self.zero_frames = 0
        self.bridge_ready = False
        self.bridge_seen_at = None
        self.sent = [0.0, 0.0, 0.0]
        self.events = deque(maxlen=20)
        self.last_tick = time.monotonic()
        self.pub = self.create_publisher(TwistStamped, '/prerobot/cmd_vel', 1)
        self.create_subscription(Bool, '/prerobot/bridge_ready', self.on_bridge, 10)
        self.create_subscription(TwistStamped, '/prerobot/sent_cmd', self.on_sent, 10)
        self.create_subscription(Float32MultiArray, '/prerobot/usb_rx', self.on_raw, 100)
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

    def start_recording(self):
        self.bag = SessionBag(self.bag_directory)
        self.bag_stop_at = None
        self.last_bag_path = str(self.bag.path)
        try:
            self.bag.sample('/prerobot/recording_event', {'event': 'start'})
        except Exception:
            self.bag.close()
            self.bag = None
            raise
        self.event(f'recording started: {self.bag.path}')

    def finish_recording(self, reason):
        if self.bag is None:
            return
        bag, self.bag = self.bag, None
        self.bag_stop_at = None
        try:
            bag.sample('/prerobot/recording_event', {'event': 'stop', 'reason': reason})
        except Exception as exc:
            self.get_logger().error(f'rosbag stop marker failed: {exc}')
        try:
            bag.close()
            self.event(f'recording saved: {bag.path}')
        except Exception as exc:
            self.get_logger().error(f'rosbag close failed: {exc}')
            self.event(f'recording close failed: {exc}')

    def record(self, topic, message, sample=False):
        if self.bag is None:
            return
        try:
            if sample:
                self.bag.sample(topic, message)
            else:
                self.bag.write(topic, message)
        except Exception as exc:
            self.get_logger().error(f'rosbag write failed: {exc}')
            self.active = False
            self.mode = 'recording_error'
            self.finish_recording('write_error')

    def on_bridge(self, message):
        with self.lock:
            if self.bridge_ready and not message.data and self.active:
                self.active = False
                self.mode = 'usb_lost'
                self.event('USB lost; command transmission stopped')
                self.finish_recording('usb_lost')
            self.bridge_ready = message.data
            self.bridge_seen_at = time.monotonic()

    def bridge_is_ready(self, now):
        return self.bridge_ready and self.bridge_seen_at is not None and now - self.bridge_seen_at < 0.2

    def on_sent(self, message):
        with self.lock:
            self.sent = [message.twist.linear.x, message.twist.linear.y,
                         message.twist.angular.z]
            self.record('/prerobot/sent_cmd', message)

    def on_raw(self, message):
        with self.lock:
            self.record('/prerobot/usb_rx', message)
            result = self.telemetry.raw(message.layout.data_offset, list(message.data))
            if result is not None:
                kind, samples = result
                topic = '/prerobot/chart_wheel' if kind == 'wheel' else '/prerobot/chart_current'
                for sample in samples if kind == 'wheel' else [samples]:
                    self.record(topic, sample, sample=True)

    def check_owner(self, token):
        if not self.token or token != self.token or time.monotonic() >= self.lease_until:
            raise ValueError('control lease expired or token invalid')

    def request(self, action, data):
        with self.lock:
            now = time.monotonic()
            if action == 'claim':
                if self.owner and now < self.lease_until:
                    raise ValueError(f'control held by {self.owner}')
                self.finish_recording('control_replaced')
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
                if not self.bridge_is_ready(now):
                    raise ValueError('USB bridge not ready')
                if self.mode == 'halting':
                    raise ValueError('wait for zero-command recording to finish')
                if self.bag_stop_at is not None:
                    self.finish_recording('halt_complete')
                if self.bag is None:
                    try:
                        self.start_recording()
                    except Exception as exc:
                        raise ValueError(f'录包启动失败，未发送目标：{exc}') from exc
                self.motion.set_target(*values)
                self.active = True
                self.zero_frames = 0
                self.mode = 'tracking'
                self.lease_until = now + self.lease_seconds
                self.event(f'target {vx:.3f} {vy:.3f} {wz:.3f}')
            elif action == 'stop':
                if not self.bridge_is_ready(now):
                    raise ValueError('USB bridge not ready')
                self.motion.set_target(0.0, 0.0, 0.0, self.motion.amax, self.motion.jerkmax)
                self.active = True
                self.zero_frames = 5
                self.mode = 'stopping'
                self.event('operator stop')
            elif action == 'halt':
                if not self.bridge_is_ready(now):
                    raise ValueError('USB bridge not ready')
                self.motion.halt()
                self.active = True
                self.zero_frames = 5
                self.mode = 'halting'
                self.event('operator halt')
            elif action == 'release':
                self.active = False
                self.finish_recording('control_released')
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
            'usb_ready': self.bridge_is_ready(now),
            'transmitting': self.active and self.bridge_is_ready(now),
            'target': self.motion.target[:],
            'command': self.motion.v[:],
            'sent': self.sent[:],
            'limits': self.limits,
            'events': list(self.events),
            'recording': self.bag is not None,
            'recording_path': str(self.bag.path) if self.bag else self.last_bag_path,
        }

    def tick(self):
        with self.lock:
            now = time.monotonic()
            dt = min(0.05, now - self.last_tick)
            self.last_tick = now
            if self.bag_stop_at is not None and now >= self.bag_stop_at:
                self.finish_recording('halt_complete')
            if self.owner and now >= self.lease_until:
                self.owner = None
                self.token = None
                self.active = False
                self.mode = 'control_lost'
                self.event('control lease lost; transmission stopped')
                self.finish_recording('control_lost')
            if self.active and not self.bridge_is_ready(now):
                self.active = False
                self.mode = 'usb_lost'
                self.event('USB bridge status expired; transmission stopped')
                self.finish_recording('usb_lost')
            if not self.active:
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
            self.record('/prerobot/cmd_vel', message)
            # Plot only commands that this controller actually published.
            sample = self.telemetry.command(self.motion.v[0], self.motion.v[1], now)
            if sample is not None:
                self.record('/prerobot/chart_command', sample, sample=True)
            if self.mode in ('stopping', 'halting') and self.zero_frames <= 0:
                completed_mode = self.mode
                self.active = False
                self.mode = 'idle'
                self.event('explicit stop complete')
                if completed_mode == 'halting' and self.bag is not None:
                    self.bag_stop_at = now + 0.2

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join()
        with self.lock:
            self.finish_recording('controller_shutdown')


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
            parsed = urlsplit(self.path)
            if parsed.path == '/api/telemetry':
                try:
                    query = parse_qs(parsed.query)
                    wheel = int(query.get('wheel', ['0'])[0])
                    since = max(0, int(query.get('since', ['0'])[0]))
                    self.respond(200, controller.telemetry.snapshot(wheel, since))
                except ValueError as exc:
                    self.respond(400, {'error': str(exc)})
                return
            if self.path == '/api/status':
                with controller.lock:
                    self.respond(200, controller.snapshot())
                return
            files = {'/': ('index.html', 'text/html'),
                     '/app.css': ('app.css', 'text/css'),
                     '/app.js': ('app.js', 'application/javascript'),
                     '/charts.js': ('charts.js', 'application/javascript')}
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
