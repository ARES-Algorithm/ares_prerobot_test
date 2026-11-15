"""Local live chart sidecar. Proxies existing controls; subscribes read-only to ROS."""
import argparse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, urlsplit
from urllib.request import Request, ProxyHandler, build_opener

from .telemetry import LiveTelemetry


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, body, content_type='application/json; charset=utf-8'):
        if not isinstance(body, bytes):
            body = json.dumps(body, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlsplit(self.path)
        if parsed.path == '/api/telemetry':
            try:
                query = parse_qs(parsed.query)
                data = self.server.telemetry.snapshot(int(query.get('wheel', ['0'])[0]),
                                                      max(0, int(query.get('since', ['0'])[0])))
                self.reply(200, data)
            except ValueError as exc:
                self.reply(400, {'error': str(exc)})
        elif parsed.path == '/api/status':
            self.proxy('GET', parsed.path)
        elif parsed.path in ('/', '/app.js', '/charts.js', '/app.css'):
            name = 'index.html' if parsed.path == '/' else parsed.path[1:]
            kind = 'text/html' if name.endswith('html') else 'text/javascript' if name.endswith('js') else 'text/css'
            self.reply(200, (self.server.assets / name).read_bytes(), kind + '; charset=utf-8')
        else:
            self.reply(404, {'error': 'Not found'})

    def do_POST(self):
        allowed = ('claim', 'release', 'command', 'stop', 'halt', 'heartbeat')
        if self.path not in tuple('/api/' + a for a in allowed):
            self.reply(404, {'error': 'Not found'})
            return
        # Browser requests must originate from this page, not another website.
        origin = self.headers.get('Origin')
        if origin and urlsplit(origin).netloc != self.headers.get('Host'):
            self.reply(403, {'error': 'Cross-origin control denied'})
            return
        self.proxy('POST', self.path)

    def proxy(self, method, path):
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if length < 0 or length > 16384:
                self.reply(413, {'error': 'Request too large'})
                return
            body = self.rfile.read(length) if method == 'POST' else None
            request = Request(self.server.upstream + path, data=body, method=method,
                              headers={'Content-Type': 'application/json'})
            with self.server.opener.open(request, timeout=2) as response:
                self.reply(response.status, response.read())
        except HTTPError as exc:
            self.reply(exc.code, exc.read())
        except (URLError, TimeoutError, OSError, ValueError):
            self.reply(502, {'error': '原控制服务不可用；检查 --control-url'})

    def log_message(self, *args):
        pass


def make_server(telemetry, host, port, upstream, assets=None):
    if assets is None:
        assets = Path(__file__).parent / 'web'
        if not assets.exists():
            from ament_index_python.packages import get_package_share_directory
            assets = Path(get_package_share_directory('ares_prerobot_test')) / 'web'
    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    server.telemetry, server.assets = telemetry, Path(assets)
    server.upstream = upstream.rstrip('/')
    server.opener = build_opener(ProxyHandler({}))
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--control-url', default='http://127.0.0.1:8766')
    parser.add_argument('--radius', type=float, default=.04)
    parser.add_argument('--rpm-ratio', type=float, default=1.)
    parser.add_argument('--window', type=float, default=.24)
    parser.add_argument('--command-topic', default='/prerobot/cmd_vel')
    parser.add_argument('--raw-topic', default='/prerobot/usb_rx')
    parser.add_argument('--web-only', action='store_true', help='serve UI without ROS; no invented data')
    args = parser.parse_args()
    telemetry = LiveTelemetry(args.radius, args.rpm_ratio, args.window)
    server = make_server(telemetry, args.host, args.port, args.control_url)
    node = None
    if not args.web_only:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import Float32MultiArray
        rclpy.init(args=[])
        node = Node('prerobot_live_charts')
        node.create_subscription(TwistStamped, args.command_topic,
            lambda m: telemetry.command(m.twist.linear.x, m.twist.linear.y), 100)
        node.create_subscription(Float32MultiArray, args.raw_topic,
            lambda m: telemetry.raw(m.layout.data_offset, list(m.data)), 200)
        threading.Thread(target=partial(rclpy.spin, node), daemon=True).start()
    print(f'Live charts: http://{args.host}:{server.server_port} (ROS={not args.web_only})', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if node:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
