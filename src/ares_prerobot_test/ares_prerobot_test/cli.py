"""Terminal client for the shared chassis controller."""
import argparse
import json
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def call(base, action, data=None):
    request = Request(base + '/api/' + action,
        data=json.dumps(data or {}).encode(),
        headers={'Content-Type': 'application/json'})
    try:
        with urlopen(request, timeout=2) as response:
            return json.load(response)
    except HTTPError as error:
        detail = json.load(error)
        raise RuntimeError(detail.get('error', str(error))) from error


def main():
    parser = argparse.ArgumentParser(description='Interactive chassis control')
    parser.add_argument('--url', default='http://127.0.0.1:8766')
    args = parser.parse_args()
    base = args.url.rstrip('/')
    token = call(base, 'claim', {'client': 'CLI'})['token']
    done = threading.Event()

    def renew():
        while not done.wait(0.4):
            try:
                call(base, 'heartbeat', {'token': token})
            except (RuntimeError, URLError):
                done.set()
                print('\n控制连接已丢失；不再发送指令。', flush=True)

    thread = threading.Thread(target=renew, daemon=True)
    thread.start()
    print('已取得控制权。输入 vx vy wz amax jerkmax；或 status / stop / halt / quit')
    try:
        while not done.is_set():
            line = input('chassis> ').strip()
            if not line:
                continue
            try:
                if line == 'status':
                    with urlopen(base + '/api/status', timeout=2) as response:
                        print(json.dumps(json.load(response), ensure_ascii=False, indent=2))
                elif line in ('stop', 'halt'):
                    call(base, line, {'token': token})
                    print('已发送' + ('受限减速' if line == 'stop' else '立即零目标') + '请求。')
                elif line == 'quit':
                    try:
                        call(base, 'halt', {'token': token})
                        time.sleep(0.12)
                    except RuntimeError:
                        pass
                    break
                else:
                    values = [float(value) for value in line.replace(',', ' ').split()]
                    if len(values) != 5:
                        print('需要 5 个数：vx vy wz amax jerkmax')
                        continue
                    data = dict(zip(('vx', 'vy', 'wz', 'amax', 'jerkmax'), values))
                    data['token'] = token
                    call(base, 'command', data)
                    print('目标已更新。')
            except (ValueError, RuntimeError) as error:
                print(error)
            except URLError:
                done.set()
                print('控制服务失联；停止发送。')
    except KeyboardInterrupt:
        print('\n控制端中断；不发送零速度。')
    finally:
        done.set()
        try:
            call(base, 'release', {'token': token})
        except (RuntimeError, URLError):
            pass
        thread.join(timeout=1)
