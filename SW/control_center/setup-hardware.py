#!/usr/bin/env python3
"""Prepare both configured CAN links and the Livox Ethernet profile."""
import argparse
from datetime import datetime
import ipaddress
import json
import os
from pathlib import Path
import pwd
import re
import shutil
import subprocess

from lidar_network import bindings
from can_devices import devices

ROOT = Path(__file__).resolve().parent


def run(command, timeout=20):
    result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(' '.join(command) + ': ' + (result.stderr or result.stdout).strip())
    return result.stdout


def prepare_usb_can(config, config_path, backup):
    settings = config.get('can', {})
    if not any('usb_serial' in settings.get(role, {}) for role in ('tracer', 'piper')):
        return
    run(['modprobe', 'gs_usb'])
    run(['udevadm', 'settle', '--timeout=10'])
    connected = [entry for entry in devices() if entry['usb_path']]
    for entry in connected:
        print('USB CAN: ' + entry['interface'] + ' / ' + entry['usb_path'] + ' / serial=' + entry['usb_serial'], flush=True)
    if len(connected) != 2:
        raise ValueError(f'USB CAN {len(connected)}개 감지 · Tracer와 팔용 USB CAN 두 개가 필요합니다. 내장 can0/can1은 대신 사용하지 않습니다.')
    serials = [entry['usb_serial'] for entry in connected]
    if any(not serial for serial in serials) or len(set(serials)) != 2:
        raise ValueError('두 USB CAN의 시리얼이 없거나 같습니다. 자동으로 역할을 배정할 수 없습니다.')
    tracer, piper = settings['tracer'], settings['piper']
    if not tracer.get('usb_serial'):
        raise ValueError('Tracer USB 시리얼이 필요합니다. --tracer-serial SERIAL로 지정하세요.')
    if tracer['usb_serial'] not in serials:
        raise ValueError('등록된 Tracer USB CAN을 찾지 못했습니다: ' + tracer['usb_serial'])
    if not piper.get('usb_serial'):
        piper['usb_serial'] = next(serial for serial in serials if serial != tracer['usb_serial'])
    if piper['usb_serial'] not in serials or piper['usb_serial'] == tracer['usb_serial']:
        raise ValueError('등록된 팔 USB CAN 시리얼이 연결된 장치와 일치하지 않습니다.')
    mapping = {}
    for role, target, rate in [('tracer', 'can_tracer', 500000), ('piper', 'can_arm', 1000000)]:
        entry = next(device for device in connected if device['usb_serial'] == settings[role]['usb_serial'])
        if not re.fullmatch(r'[A-Za-z0-9_-]+', entry['usb_serial']):
            raise ValueError('udev 이름 고정에 사용할 수 없는 USB 시리얼입니다.')
        settings[role].update(interface=target, bitrate=rate)
        mapping[target] = entry
    all_interfaces = {entry['interface'] for entry in devices()}
    for target in mapping:
        if target in all_interfaces and target not in [entry['interface'] for entry in connected]:
            raise ValueError('다른 장치가 대상 CAN 이름을 사용 중입니다: ' + target)
    rules = Path('/etc/udev/rules.d/80-can-usb.rules')
    if rules.exists():
        shutil.copy2(rules, backup / rules.name)
    rules.parent.mkdir(parents=True, exist_ok=True)
    lines = ['# Match each robot USB CAN by its unique serial; never give both the same name.']
    for target, entry in mapping.items():
        lines.append(f'SUBSYSTEM=="net", ACTION=="add", ATTR{{type}}=="280", ATTRS{{idVendor}}=="{entry["vendor"]}", ATTRS{{idProduct}}=="{entry["product_id"]}", ATTRS{{serial}}=="{entry["usb_serial"]}", NAME="{target}"')
    rules.write_text('\n'.join(lines) + '\n')
    run(['udevadm', 'control', '--reload-rules'])
    # Vacate both names first; the current Tracer may still be named can_arm.
    staging = []
    for index, (target, entry) in enumerate(mapping.items()):
        temporary = f'can_cc{os.getpid() % 100000}_{index}'
        if (Path('/sys/class/net') / temporary).exists():
            raise ValueError('임시 CAN 이름이 사용 중입니다: ' + temporary)
        snapshot = json.loads(run(['ip', '-json', 'link', 'show', 'dev', entry['interface']]))[0]
        staging.append(dict(original=entry['interface'], current=entry['interface'],
                            temporary=temporary, target=target, up='UP' in snapshot.get('flags', [])))
    try:
        for item in staging:
            run(['ip', 'link', 'set', 'dev', item['current'], 'down'])
            run(['ip', 'link', 'set', 'dev', item['current'], 'name', item['temporary']])
            item['current'] = item['temporary']
        for item in staging:
            run(['ip', 'link', 'set', 'dev', item['current'], 'name', item['target']])
            item['current'] = item['target']
    except (OSError, RuntimeError, subprocess.SubprocessError):
        # Vacate names again before restoring a partially completed swap.
        for item in staging:
            try:
                run(['ip', 'link', 'set', 'dev', item['current'], 'down'])
                if item['current'] != item['temporary']:
                    run(['ip', 'link', 'set', 'dev', item['current'], 'name', item['temporary']])
                    item['current'] = item['temporary']
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                print('CAN 이름 복구 준비 실패: ' + str(exc), flush=True)
        for item in staging:
            try:
                if item['current'] != item['original']:
                    run(['ip', 'link', 'set', 'dev', item['current'], 'name', item['original']])
                if item['up']:
                    run(['ip', 'link', 'set', 'dev', item['original'], 'up'])
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                print('CAN 이름 복구 실패: ' + str(exc), flush=True)
        raise
    if config_path.exists():
        shutil.copy2(config_path, backup / config_path.name)
    config_path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
    if os.environ.get('SUDO_UID') and os.environ.get('SUDO_GID'):
        os.chown(config_path, int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID']))
    print('USB 역할 저장: Tracer=can_tracer 500 kbps / 팔=can_arm 1 Mbps', flush=True)


def setup_can(config, config_path, backup):
    prepare_usb_can(config, config_path, backup)
    settings = config.get('can', {})
    assignments = {}
    for role, default, rate in [('tracer', 'can_tracer', 500000), ('piper', 'can_arm', 1000000)]:
        entry = settings.get(role, {})
        interface = str(entry.get('interface', default))
        bitrate = int(entry.get('bitrate', rate))
        if not re.fullmatch(r'can[A-Za-z0-9_]{1,11}', interface) or not 10000 <= bitrate <= 1000000:
            raise ValueError('CAN 이름 또는 bitrate 설정 오류: ' + role)
        if interface in assignments:
            raise ValueError('차체와 팔에 서로 다른 CAN 인터페이스가 필요합니다.')
        assignments[interface] = bitrate
    network = Path('/etc/systemd/network')
    network.mkdir(parents=True, exist_ok=True)
    for interface in set(assignments) | {'can0', 'can_arm'}:
        filename = '80-can-arm.network' if interface == 'can_arm' else f'80-{interface}.network'
        path = network / filename
        if path.exists():
            shutil.copy2(path, backup / filename)
        if interface in assignments:
            path.write_text(f'[Match]\nName={interface}\n[CAN]\nBitRate={assignments[interface]}\nRestartSec=100ms\n[Link]\nRequiredForOnline=no\n')
        elif path.exists():
            # Remove the earlier robot assignment without disabling an unrelated controller.
            path.write_text('[Match]\nName=inha_unused_can_profile\n')
    run(['systemctl', 'enable', '--now', 'systemd-networkd'])
    run(['networkctl', 'reload'])
    for interface, bitrate in assignments.items():
        if not (Path('/sys/class/net') / interface).exists():
            print('주의: CAN 장치 없음: ' + interface, flush=True)
            continue
        run(['ip', 'link', 'set', 'dev', interface, 'down'])
        run(['ip', 'link', 'set', 'dev', interface, 'type', 'can', 'bitrate', str(bitrate), 'restart-ms', '100'])
        run(['ip', 'link', 'set', 'dev', interface, 'up'])
        run(['networkctl', 'reconfigure', interface])
        print(f'CAN 연결 설정: {interface} {bitrate // 1000} kbps', flush=True)
    print('CAN 역할은 config.json의 현재 배정을 유지합니다. 실제 로봇 피드백 수신과는 별개입니다.', flush=True)


def setup_lidar(config, backup):
    settings = config.get('sensors', {}).get('mid360_lidar', {})
    owner = os.environ.get('SUDO_USER', pwd.getpwuid(os.getuid()).pw_name)
    user_home = Path(pwd.getpwnam(owner).pw_dir)
    value = str(settings.get('config_file', '~/ws_livox/src/livox_ros_driver2/config/MID360s_config.json'))
    path = Path(str(user_home) + value[1:] if value.startswith('~/') else value)
    hosts = {address for address, _ in bindings(path) if address != '0.0.0.0'}
    if len(hosts) != 1:
        raise ValueError('자동 Ethernet 설정은 Livox host IP 하나를 지원합니다.')
    address = next(iter(hosts))
    interface = str(settings.get('interface', 'eno1'))
    if not (Path('/sys/class/net') / interface).exists():
        raise ValueError('Livox Ethernet 장치 없음: ' + interface)
    prefix = int(settings.get('prefix', 24))
    network = ipaddress.IPv4Interface(f'{address}/{prefix}').network
    for entry in json.loads(path.read_text()).get('lidar_configs', []):
        if ipaddress.IPv4Address(entry['ip']) not in network:
            raise ValueError('Livox 센서 IP와 host IP의 서브넷이 다릅니다.')
    connection = str(settings.get('connection', 'livox'))
    names = run(['nmcli', '-g', 'NAME', 'connection', 'show']).splitlines()
    if connection not in names:
        run(['nmcli', 'connection', 'add', 'type', 'ethernet', 'ifname', interface, 'con-name', connection])
    kind = run(['nmcli', '-g', 'connection.type', 'connection', 'show', 'id', connection]).strip()
    if kind != '802-3-ethernet':
        raise ValueError('Livox 전용 연결이 Ethernet 프로필이 아닙니다: ' + connection)
    profile = Path('/etc/NetworkManager/system-connections') / (connection + '.nmconnection')
    if profile.is_file():
        shutil.copy2(profile, backup / profile.name)
    run(['nmcli', 'connection', 'modify', 'id', connection, 'connection.interface-name', interface,
         'connection.autoconnect', 'yes', 'ipv4.method', 'manual', 'ipv4.addresses', f'{address}/{prefix}',
         'ipv4.gateway', '', 'ipv4.dns', '', 'ipv4.never-default', 'yes', 'ipv6.method', 'disabled'])
    run(['nmcli', '--wait', '10', 'connection', 'up', 'id', connection], timeout=15)
    print(f'MID-360 Ethernet 설정: {connection} / {interface} / {address}/{prefix}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can', action='store_true')
    parser.add_argument('--lidar', action='store_true')
    parser.add_argument('--config', default=str(ROOT / 'config.json'))
    parser.add_argument('--tracer-serial')
    parser.add_argument('--piper-serial')
    args = parser.parse_args()
    if os.geteuid() != 0:
        raise SystemExit('setup-can.sh 또는 setup-hardware.sh로 실행하세요 (sudo 필요).')
    path = Path(args.config).resolve()
    config = json.loads((path if path.exists() else ROOT / 'config.example.json').read_text())
    for role, serial in [('tracer', args.tracer_serial), ('piper', args.piper_serial)]:
        if serial:
            config.setdefault('can', {}).setdefault(role, {})['usb_serial'] = serial
    backup = Path('/var/backups/inha-control-center') / (datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + str(os.getpid()))
    backup.mkdir(parents=True, exist_ok=True)
    if args.can:
        setup_can(config, path, backup)
    if args.lidar:
        setup_lidar(config, backup)
    print('하드웨어 설정 저장 완료 · 백업: ' + str(backup), flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
        raise SystemExit('하드웨어 설정 실패: ' + str(exc))
