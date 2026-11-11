"""Write rover diagnostics to local JSON Lines files and serve a loopback viewer."""
import argparse
from datetime import datetime
from http.client import HTTPConnection, HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import ip_address, ip_network
import json
from pathlib import Path
import socket
from threading import Lock, Thread
from time import monotonic, sleep
from urllib.parse import urlsplit


TOOLS = Path(__file__).resolve().parent
PROJECT = TOOLS.parent / 'project'
LOG_DIR = Path('D:/RoverDiagnostics/logs')
PORT = 8767
BASE = '192.168.4.1'
CAMERA = 'rover-camera.local'
PRIVATE_NETS = tuple(map(ip_network, ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16')))
STATIC = {
    **{name: 'text/html; charset=utf-8' for name in
       ('gimbal-3d-scan.html', 'gimbal-radar-standalone.html')},
    **{name + '.js': 'application/javascript; charset=utf-8' for name in
       ('three.min', 'OrbitControls.umd', 'gimbal-model', 'scan-snapshot',
        'vehicle-route', 'route-log', 'wheel-speed-panel', 'camera-panel', 'rover-demo')},
}


def camera_host(value):
    value = value.strip()
    if not value:
        return ''
    url = urlsplit(value if '://' in value else 'http://' + value)
    host = url.hostname
    if (url.scheme != 'http' or not host or url.port not in (None, 80)
            or url.username is not None or url.password is not None
            or url.path not in ('', '/', '/capture', '/status') or url.query or url.fragment):
        raise ValueError('请填写摄像头的局域网 IP 或 rover-camera.local')
    if host == CAMERA:
        return host
    try:
        address = ip_address(host)
        if host != BASE and any(address in network for network in PRIVATE_NETS):
            return host
    except ValueError:
        pass
    raise ValueError('请填写摄像头的局域网 IP 或 rover-camera.local')


def fields(value, keys):
    if not isinstance(value, dict):
        return {}
    return {key: value[key] for key in keys.split()
            if key in value and isinstance(value[key], (bool, int, str))}


class RoverConnection(HTTPConnection):
    def connect(self):
        try:
            address = ip_address(self.host)
        except ValueError:
            return super().connect()
        # Windows name resolution for rover-camera.local can delay getaddrinfo
        # in the other thread too. Connect to numeric rover addresses directly.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((str(address), self.port))


def sample(host, path):
    started = monotonic()
    conn = RoverConnection(host, timeout=0.9)
    try:
        conn.request('GET', path, headers={'Connection': 'close', 'Accept': 'application/json'})
        with conn.getresponse() as response:
            if response.status != 200:
                raise ValueError('HTTP ' + str(response.status))
            body = response.read(8193)
        if len(body) > 8192:
            raise ValueError('响应过长')
        data = json.loads(body)
        required = 'online' if path == '/api/telemetry' else 'cameraReady'
        if not isinstance(data, dict) or not isinstance(data.get(required), bool):
            raise ValueError('遥测格式不匹配')
        if path == '/api/telemetry':
            result = fields(data, 'online link sequence ageMs handheldUdpOnline uptimeMs '
                            'maxUdpGapMs maxForwardGapMs skippedUdp maxHttpPollUs')
            vehicle = fields(data.get('vehicle'), 'online mode state')
            pwm = (data.get('vehicle') or {}).get('pwm') if vehicle else None
            if isinstance(pwm, list) and len(pwm) == 4 and all(type(p) is int for p in pwm):
                vehicle['pwm'] = pwm
            result['vehicle'] = vehicle or None
            result['controlDiagnostics'] = fields(data.get('controlDiagnostics'),
                                                  'hold uartRxFaults maxMotorServiceGapMs') or None
        else:
            result = fields(data, 'cameraReady cameraError version rssi maxFps')
        result['ok'] = True
    except (OSError, HTTPException, ValueError) as exc:
        result = {'ok': False, 'error': str(exc)[:140] or type(exc).__name__}
    finally:
        conn.close()
    result['requestMs'] = round((monotonic() - started) * 1000)
    return result


class Recorder:
    def __init__(self, directory):
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.settings = directory / 'camera.txt'
        self.camera = camera_host(self.settings.read_text(encoding='utf-8')) if self.settings.is_file() else CAMERA
        self.latest = {}
        self.file = None
        self.error = None
        self.lock = Lock()

    def record(self, source, value, save=True):
        with self.lock:
            now = datetime.now().astimezone()
            entry = {'time': now.isoformat(timespec='milliseconds'), 'source': source, **value}
            self.latest[source] = entry
            if save:
                path = self.directory / ('rover-' + now.strftime('%Y-%m-%d') + '.jsonl')
                try:
                    with path.open('a', encoding='utf-8', newline='\n') as stream:
                        stream.write(json.dumps(entry, ensure_ascii=False, separators=(',', ':')) + '\n')
                    self.file = path
                    self.error = None
                except OSError as exc:
                    self.error = str(exc)
                    return False
        return True

    def set_camera(self, host):
        with self.lock:
            if host != self.camera:
                temporary = self.settings.with_suffix('.tmp')
                temporary.write_text(host, encoding='utf-8')
                temporary.replace(self.settings)
                self.camera = host
                self.latest.pop('camera', None)

    def status(self):
        with self.lock:
            return {'cameraHost': self.camera, 'file': str(self.file) if self.file else None,
                    'error': self.error, 'latest': self.latest.copy()}


def collect(recorder, source, path, interval):
    previous_error = None
    previous_host = None
    last_save = 0
    while True:
        host = BASE if source == 'base' else recorder.camera
        if host:
            began = monotonic()
            value = sample(host, path)
            if source == 'base' or host == recorder.camera:
                error = value.get('error')
                save = value['ok'] or error != previous_error or host != previous_host or began - last_save >= 10
                recorder.record(source, value, save)
                if save:
                    last_save = began
                previous_error, previous_host = error, host
            sleep(interval if value['ok'] else max(2, interval))
        else:
            previous_host = None
            sleep(1)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def setup(self):
        super().setup()
        self.connection.settimeout(3)

    def reply(self, status, body, mime):
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def json(self, status, data):
        self.reply(status, json.dumps(data, ensure_ascii=False).encode('utf-8'), 'application/json; charset=utf-8')

    def do_GET(self):
        if self.headers.get('Host') != self.server.local_host:
            return self.send_error(403)
        path = self.path.partition('?')[0]
        if path == '/api/status':
            return self.json(200, self.server.recorder.status())
        if path == '/':
            file, mime = TOOLS / 'rover-diagnostics.html', 'text/html; charset=utf-8'
        elif path[1:] in STATIC:
            file, mime = PROJECT / path[1:], STATIC[path[1:]]
        else:
            return self.send_error(404)
        try:
            body = file.read_bytes()
        except OSError:
            return self.send_error(404)
        self.reply(200, body, mime)

    def do_POST(self):
        if (self.headers.get('Host') != self.server.local_host or
                self.headers.get('Origin') != 'http://' + self.server.local_host):
            return self.send_error(403)
        if self.headers.get_content_type() != 'application/json':
            return self.send_error(415)
        recorder = self.server.recorder
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 1024:
                return self.send_error(413)
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError('需要 JSON 对象')
            if self.path == '/api/camera' and isinstance(data.get('host'), str):
                recorder.set_camera(camera_host(data['host']))
            elif self.path == '/api/mark' and isinstance(data.get('note'), str):
                if not recorder.record('mark', {'note': data['note'].strip()[:120] or '出现卡顿'}):
                    return self.json(500, {'error': recorder.error})
            elif self.path == '/api/video' and data.get('event') in ('start', 'frame', 'error', 'stale', 'stop'):
                value = fields(data, 'event page requestedFps fetchMs frameAgeMs frameGapMs profile failures')
                if isinstance(data.get('reason'), str):
                    value['reason'] = data['reason'][:150]
                if data['event'] == 'start' and isinstance(data.get('camera'), str):
                    try:
                        value['camera'] = camera_host(data['camera'])
                        recorder.set_camera(value['camera'])
                    except ValueError:
                        pass  # Video logging still works if the probe address is unsupported.
                if not recorder.record('video', value):
                    return self.json(500, {'error': recorder.error})
            else:
                return self.send_error(404)
        except (OSError, ValueError) as exc:
            return self.json(400, {'error': str(exc)[:180]})
        self.json(200, {'ok': True})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=LOG_DIR)
    args = parser.parse_args()
    with ThreadingHTTPServer(('127.0.0.1', PORT), Handler) as server:
        server.local_host = '127.0.0.1:' + str(PORT)
        server.recorder = Recorder(args.log_dir)
        server.recorder.record('recorder', {'event': 'start'})
        for source, path, seconds in (('base', '/api/telemetry', 1), ('camera', '/status', 4)):
            Thread(target=collect, args=(server.recorder, source, path, seconds), daemon=True).start()
        print('Diagnostics: http://' + server.local_host + '/  |  ' + str(args.log_dir), flush=True)
        server.serve_forever()


if __name__ == '__main__':
    main()
