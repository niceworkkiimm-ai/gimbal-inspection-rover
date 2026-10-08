"""Read-only source export, public sanitization, validation and Git backup. Python 3.7+."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
LOCAL = ROOT / '.local'
CONFIG = LOCAL / 'config.json'
STATE = LOCAL / 'sync-state.json'
DIRS = {'User', 'Hardware', 'Library', 'Start', 'System', 'ESP01S_Base_UDP',
        'NodeMCU_Handheld_MPU_Car', 'ESP32CAM_Camera', 'ESP32S3_Camera',
        'ESP01S_Handheld_MPU_UDP', 'ESP8266_Firmware', 'MiniProgram', 'web-dashboard', 'tests'}
EXTS = {'.c', '.h', '.cpp', '.ino', '.s', '.js', '.cjs', '.html', '.css', '.json',
        '.md', '.ps1', '.csv', '.png', '.wxml', '.wxss', '.uvprojx'}
SKIP = {'.git', '.agents', '.codex', '.vscode', 'node_modules', 'objects', 'listings',
        'rte', 'tmp', '__pycache__', '.local'}
PASSWORD_FIELDS = {
    'ESP01S_Base_UDP/ESP01S_Base_UDP.ino': 'kApPassword',
    'NodeMCU_Handheld_MPU_Car/NodeMCU_Handheld_MPU_Car.ino': 'kApPassword',
    'ESP01S_Handheld_MPU_UDP/ESP01S_Handheld_MPU_UDP.ino': 'kApPassword',
    'ESP8266_Firmware/wifi_bridge/wifi_bridge.ino': 'AP_PASSWORD',
    'ESP32CAM_Camera/CameraConfig.h': 'CAMERA_WIFI_PASSWORD',
    'ESP32S3_Camera/CameraConfig.h': 'CAMERA_WIFI_PASSWORD',
}
EXAMPLE = ('#pragma once\n// Copy to secrets.h and set your own Wi-Fi password.\n'
           '// Base, handheld and camera must use the SAME password (8-63 characters).\n'
           '#define GIMBAL_WIFI_PASSWORD "CHANGE_ME_BEFORE_FLASH"\n')

def sha(data):
    return hashlib.sha256(data).hexdigest()

def config():
    if not CONFIG.exists():
        raise RuntimeError('Run tools/setup.ps1 first, or create .local/config.json from the example.')
    return json.loads(CONFIG.read_text(encoding='utf-8-sig'))

def within(path, parent):
    p, base = Path(path).resolve(), Path(parent).resolve()
    if p == base or base not in p.parents:
        raise RuntimeError('Unsafe destination outside expected directory.')
    # Windows directory junctions/reparse points must not redirect writes.
    cursor = p
    while cursor != base.parent:
        if cursor.exists() and getattr(cursor.lstat(), 'st_file_attributes', 0) & 0x400:
            raise RuntimeError('Reparse point in destination path: ' + str(cursor))
        cursor = cursor.parent
    return p

def allowed(rel):
    return (not any(p.lower() in SKIP or p.lower().startswith('.arduino') for p in rel.parts)
            and rel.name not in {'secrets.h', 'project.private.config.json'}
            and rel.suffix.lower() in EXTS
            and (len(rel.parts) == 1 or rel.parts[0] in DIRS))

def read_source(source):
    files = {}
    for folder, dirs, names in os.walk(str(source), followlinks=False):
        base = Path(folder)
        dirs[:] = [d for d in dirs if d.lower() not in SKIP and not d.startswith('.arduino')
                   and (base != source or d in DIRS)
                   and not getattr((base / d).lstat(), 'st_file_attributes', 0) & 0x400]
        for name in names:
            p = base / name
            rel = p.relative_to(source)
            if allowed(rel):
                if p.is_symlink() or getattr(p.lstat(), 'st_file_attributes', 0) & 0x400:
                    raise RuntimeError('Refusing linked source: ' + str(rel))
                files[rel.as_posix()] = p.read_bytes()
    for required in ['User/main.c', 'Gimbal_STLink.uvprojx'] + list(PASSWORD_FIELDS):
        if required not in files:
            raise RuntimeError('Missing expected source file: ' + required)
    return files

def prepare(files):
    exports, private, secrets = {}, {}, []
    for name, field in PASSWORD_FIELDS.items():
        text = files[name].decode('utf-8-sig')
        pattern = r'(\b' + re.escape(field) + r'\s*(?:\[\s*\])?\s*=\s*)("(?:\\.|[^"\\])*")(\s*;)'
        hits = list(re.finditer(pattern, text))
        if len(hits) != 1:
            raise RuntimeError('Password declaration changed; review export rule: ' + name)
        literal = hits[0].group(2)
        value = json.loads(literal)
        if len(value) < 8:
            raise RuntimeError('Unexpected Wi-Fi secret; review locally: ' + name)
        secrets.append(value)
        text = re.sub(pattern, lambda m: m.group(1) + 'GIMBAL_WIFI_PASSWORD' + m.group(3), text)
        exports[name] = '#include "secrets.h"\n' + text
        directory = str(Path(name).parent).replace('\\', '/')
        private[directory + '/secrets.h'] = '#pragma once\n#define GIMBAL_WIFI_PASSWORD ' + literal + '\n'
        exports[directory + '/secrets.example.h'] = EXAMPLE
    for name, raw in files.items():
        if name in exports:
            continue
        if Path(name).suffix.lower() == '.png':
            exports[name] = raw
        else:
            # Retain original legacy comment encodings byte-for-byte.
            text = raw.decode('utf-8', errors='surrogateescape')
            if name.endswith('/project.config.json'):
                data = json.loads(text)
                data['appid'] = 'touristappid'
                text = json.dumps(data, ensure_ascii=False, indent=2) + '\n'
            exports[name] = text
    for name, content in list(exports.items()):
        if isinstance(content, str):
            for secret in set(secrets):
                # Token boundaries avoid corrupting digit alphabets or numeric data.
                content = re.sub(r'(?<![A-Za-z0-9])' + re.escape(secret) + r'(?![A-Za-z0-9])',
                                 '<LOCAL_WIFI_PASSWORD>', content)
            content = re.sub(r'C:\\Users\\[^\\\s`"<>]+', r'C:\\Users\\YOUR_USER', content)
            content = re.sub(r'C:/Users/[^/\s`"<>]+', 'C:/Users/YOUR_USER', content)
            exports[name] = content.encode('utf-8', errors='surrogateescape')
    return exports, private, secrets

def run(args, cwd=ROOT, capture=False):
    options = dict(cwd=str(cwd), check=True, env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
    if capture:
        options.update(stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    result = subprocess.run([str(a) for a in args], **options)
    return result.stdout.decode('utf-8', errors='replace').strip() if capture else ''

def sync(cfg):
    source = Path(cfg['source']).resolve()
    if source == ROOT or ROOT in source.parents or source in ROOT.parents:
        raise RuntimeError('Source and Git repository must be separate directories.')
    if os.name == 'nt' and ROOT.drive.upper() != 'D:':
        raise RuntimeError('This backup repository must live on D: as requested.')
    files = read_source(source)
    exports, private, secrets = prepare(files)
    hashes = {name: sha(raw) for name, raw in files.items()}
    if hashes != {name: sha(raw) for name, raw in read_source(source).items()}:
        raise RuntimeError('Source is still being edited; retry when editing is finished.')
    old = json.loads(STATE.read_text(encoding='utf-8')) if STATE.exists() else {'exports': {}}
    managed = {'project/' + name: raw for name, raw in exports.items()}
    # Refuse to overwrite edits made directly to a previously exported file.
    for name, expected in old['exports'].items():
        p = within(ROOT / name, ROOT / 'project')
        if p.exists() and sha(p.read_bytes()) != expected:
            raise RuntimeError('Local exported file was edited; preserve/merge it first: ' + name)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    backup_base = Path(cfg['backup_root']).resolve()
    if os.name == 'nt' and backup_base.drive.upper() != 'D:':
        raise RuntimeError('Backup destination must be on D:.')
    changed = 0
    for name in sorted(set(old['exports']) | set(managed)):
        target = within(ROOT / name, ROOT / 'project')
        new = managed.get(name)
        existing = target.read_bytes() if target.exists() else None
        if existing == new:
            continue
        if existing is not None:
            saved = within(backup_base / stamp / name, backup_base)
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(target), str(saved))
        if new is None:
            target.unlink()  # Only previously managed files, after the checked backup.
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(new)
        changed += 1
    for name, text in private.items():
        p = within(ROOT / 'project' / name, ROOT / 'project')
        if not p.exists():
            p.write_text(text, encoding='utf-8')
    # Re-read to detect edits during the export; never write into the original tree.
    if hashes != {name: sha(raw) for name, raw in read_source(source).items()}:
        raise RuntimeError('Source changed during sync. Run again after editing has finished.')
    LOCAL.mkdir(exist_ok=True)
    STATE.write_text(json.dumps({'source_sha256': hashes,
                                'exports': {name: sha(raw) for name, raw in managed.items()}},
                               ensure_ascii=False, indent=2), encoding='utf-8')
    print('Sync: {} source files, {} public files, {} changes. Original hashes unchanged.'.format(
        len(files), len(managed), changed))
    return secrets

def scan_public(secrets, staged=False):
    if staged:
        names = run(['git', 'ls-files', '-z'], capture=True).split('\x00')
    else:
        names = []
        for folder, dirs, files in os.walk(str(ROOT)):
            dirs[:] = [d for d in dirs if d.lower() not in SKIP and not d.startswith('.arduino')]
            for file in files:
                p = Path(folder) / file
                rel = p.relative_to(ROOT).as_posix()
                if file != 'secrets.h' and p.suffix.lower() not in {'.log', '.axf', '.o', '.d', '.crf', '.map', '.lst'}:
                    names.append(rel)
    failures = []
    key_pattern = re.compile(r'gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{25,}|'
                             r'sk-[A-Za-z0-9_-]{24,}|-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----')
    for name in filter(None, names):
        if any(part in {'.local', '.git'} for part in Path(name).parts) or Path(name).name == 'secrets.h':
            failures.append(name + ': private file tracked')
            continue
        raw = subprocess.check_output(['git', 'show', ':' + name], cwd=str(ROOT)) if staged else (ROOT / name).read_bytes()
        if len(raw) > 40 * 1024 * 1024:
            failures.append(name + ': unexpectedly large file')
        text = raw.decode('utf-8', errors='ignore')
        if key_pattern.search(text):
            failures.append(name + ': credential-like token')
        for secret in set(secrets):
            if re.search(r'(?<![A-Za-z0-9])' + re.escape(secret) + r'(?![A-Za-z0-9])', text):
                failures.append(name + ': local Wi-Fi password found')
                break
        if Path(name).suffix.lower() in {'.ino', '.h', '.cpp', '.c'} and re.search(
                r'\b(?:kApPassword|AP_PASSWORD|CAMERA_WIFI_PASSWORD)\s*(?:\[\s*\])?\s*=\s*"', text):
            failures.append(name + ': hardcoded Wi-Fi password')
    if failures:
        raise RuntimeError('Public export check failed:\n' + '\n'.join(failures))
    print('Public export checks passed ({} files).'.format(len(names)))

def validate(cfg):
    project = ROOT / 'project'
    tree = ET.parse(str(project / 'Gimbal_STLink.uvprojx'))
    for element in tree.findall('.//FilePath'):
        p = project / element.text.replace('\\', '/')
        if not p.is_file():
            raise RuntimeError('Missing Keil dependency: ' + element.text)
    node = cfg.get('node') or shutil.which('node')
    if not node:
        raise RuntimeError('Node.js is required for the existing webpage tests.')
    check_dir = LOCAL / 'checks'
    check_dir.mkdir(parents=True, exist_ok=True)
    for name in ['gimbal-3d-scan.html', 'gimbal-radar-standalone.html']:
        html = (project / name).read_text(encoding='utf-8')
        for src in re.findall(r'<script\b[^>]*\bsrc="([^"]+)"', html):
            if not (project / src.split('?')[0]).is_file():
                raise RuntimeError('Missing webpage dependency: ' + src)
        for i, body in enumerate(re.findall(r'<script\b[^>]*>([\s\S]*?)</script>', html)):
            if body.strip():
                temp = check_dir / (name + str(i) + '.js')
                temp.write_text(body, encoding='utf-8')
                run([node, '--check', temp])
    for p in project.glob('*.js'):
        run([node, '--check', p])
    run([node, 'tests/route-snapshot.test.cjs'], cwd=project)
    if cfg.get('keil'):
        log = LOCAL / 'keil-build.log'
        if log.exists():
            log.unlink()
        def quote(s):
            return "'" + str(s).replace("'", "''") + "'"
        command = ("$p=Start-Process -FilePath " + quote(cfg['keil']) +
                   " -ArgumentList @('-r', " + quote('"' + str(project / 'Gimbal_STLink.uvprojx') + '"') +
                   ", '-t', '\"Gimbal STLink\"', '-o', " + quote('"' + str(log) + '"') +
                   ") -WindowStyle Hidden -PassThru -Wait; exit $p.ExitCode")
        run(['powershell', '-NoProfile', '-Command', command], cwd=project)
        if not log.exists() or '0 Error(s), 0 Warning(s)' not in log.read_text(encoding='utf-8', errors='replace'):
            raise RuntimeError('Keil did not produce a clean build. Inspect .local/keil-build.log.')
        print('Keil rebuild passed: 0 errors, 0 warnings. Hardware is not flashed.')
    else:
        print('Keil not configured: MCU compilation skipped.')
    if cfg.get('arduino_cli'):
        build_root = Path(cfg['arduino_build_root']).resolve()
        if os.name == 'nt' and (build_root.drive.upper() != 'D:' or not str(build_root).isascii()):
            raise RuntimeError('Arduino build directory must be an ASCII-only D: path.')
        targets = [('base', 'esp8266:esp8266:generic:xtal=80,FlashMode=dout,FlashFreq=40', 'ESP01S_Base_UDP'),
                   ('handheld', 'esp8266:esp8266:nodemcuv2', 'NodeMCU_Handheld_MPU_Car'),
                   ('camera', 'esp32:esp32:esp32cam', 'ESP32CAM_Camera')]
        for name, fqbn, sketch in targets:
            temp = build_root / ('temp-' + name)
            temp.mkdir(parents=True, exist_ok=True)
            env = dict(os.environ, TEMP=str(temp), TMP=str(temp),
                       ARDUINO_BUILD_CACHE_PATH=str(build_root / ('cache-' + name)),
                       ARDUINO_DIRECTORIES_DOWNLOADS=str(build_root / 'downloads'))
            log = LOCAL / ('arduino-' + name + '.log')
            with log.open('wb') as stream:
                result = subprocess.run([cfg['arduino_cli'], 'compile', '--fqbn', fqbn, '--jobs', '2',
                                         '--build-path', str(build_root / name), str(project / sketch)],
                                        cwd=str(ROOT), env=env, stdout=stream, stderr=subprocess.STDOUT)
            if result.returncode:
                raise RuntimeError('Arduino build failed: ' + name + '; inspect ' + str(log))
            print('Arduino compile passed: ' + name, flush=True)
    else:
        print('Arduino CLI not configured: ESP compilation skipped.')
    print('Validation passed.')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['sync', 'validate', 'publish'])
    parser.add_argument('--message')
    args = parser.parse_args()
    cfg = config()
    if args.action == 'publish' and not (args.message and args.message.strip()):
        raise RuntimeError('Publish needs --message with a clear description of the changes.')
    secrets = []
    if args.action in {'sync', 'publish'}:
        secrets = sync(cfg)
    else:
        _, _, secrets = prepare(read_source(Path(cfg['source']).resolve()))
    scan_public(secrets)
    if args.action in {'validate', 'publish'}:
        validate(cfg)
    if args.action == 'publish':
        origin = run(['git', 'remote', 'get-url', 'origin'], capture=True)
        if re.sub(r'\.git$', '', origin.rstrip('/')) != re.sub(r'\.git$', '', cfg['remote'].rstrip('/')):
            raise RuntimeError('Remote does not match the dedicated project repository.')
        run(['git', 'add', '--all', '.'])
        scan_public(secrets, staged=True)
        diff = subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=str(ROOT))
        if diff.returncode == 1:
            run(['git', 'commit', '-m', args.message.strip()])
        elif diff.returncode != 0:
            raise RuntimeError('Unable to inspect the Git index.')
        run(['git', 'push', '-u', 'origin', 'main'])
        local = run(['git', 'rev-parse', 'HEAD'], capture=True)
        remote = run(['git', 'ls-remote', 'origin', 'refs/heads/main'], capture=True).split()[0]
        if local != remote:
            raise RuntimeError('Remote commit verification failed.')
        print('Verified GitHub main at ' + local)

if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('ERROR: ' + str(exc), file=sys.stderr)
        sys.exit(1)
