#!/usr/bin/env python3
"""Explain missing host addresses / occupied ports before starting Livox."""
import argparse
import errno
import json
import os
import socket
import subprocess

from lidar_network import bindings


def local_addresses():
    result = subprocess.run(['ip', '-json', '-4', 'address', 'show'],
                            capture_output=True, text=True, check=True, timeout=3)
    return {address['local'] for link in json.loads(result.stdout)
            for address in link.get('addr_info', [])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--connection', default='livox')
    args = parser.parse_args()
    required = bindings(args.config)
    hosts = {address for address, _ in required if address != '0.0.0.0'}
    missing = hosts - local_addresses()
    activation_error = ''
    if missing:
        try:
            result = subprocess.run(['nmcli', '--wait', '8', 'connection', 'up', 'id', args.connection],
                                    capture_output=True, text=True, timeout=10)
            if result.returncode:
                activation_error = (result.stderr or result.stdout).strip()
        except (OSError, subprocess.SubprocessError) as exc:
            activation_error = str(exc)
        missing = hosts - local_addresses()
    if missing:
        raise RuntimeError('MID-360 host IP 없음: ' + ', '.join(sorted(missing)) +
                           ' · bash ~/robocup/SW/control_center/setup-lidar.sh 실행 필요' +
                           (' · ' + activation_error if activation_error else ''))
    sockets = []
    try:
        for address, port in required:
            channel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sockets.append(channel)
            try:
                channel.bind((address, port))
            except OSError as exc:
                reason = '이미 사용 중 · 기존 Livox 드라이버 종료 필요' if exc.errno == errno.EADDRINUSE else str(exc)
                raise RuntimeError(f'MID-360 UDP bind 실패 {address}:{port}: {reason}') from exc
    finally:
        for channel in sockets:
            channel.close()
    print('MID-360 host IP와 UDP 포트 준비 완료 · 드라이버 실행', flush=True)
    command = ['ros2', 'run', 'livox_ros_driver2', 'livox_ros_driver2_node', '--ros-args',
               '-r', '__node:=livox_lidar_publisher', '-p', 'xfer_format:=0', '-p', 'multi_topic:=0',
               '-p', 'data_src:=0', '-p', 'publish_freq:=10.0', '-p', 'output_data_type:=0',
               '-p', 'frame_id:=livox_frame', '-p', 'user_config_path:=' + args.config,
               '-p', 'cmdline_input_bd_code:=livox0000000001']
    os.execvp(command[0], command)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
        raise SystemExit('MID-360 시작 실패: ' + str(exc))
