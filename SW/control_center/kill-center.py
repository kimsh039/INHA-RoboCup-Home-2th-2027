#!/usr/bin/env python3
"""Force-stop descendants of explicitly selected robot tmux sessions."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

CAN_SESSIONS = ('inha_control_center', 'robot_physical', 'tracer')


def absent(message):
    return ("can't find session" in message or 'no server running' in message or
            ('error connecting to' in message and
             ('No such file or directory' in message or 'Connection refused' in message)))


def process_info(pid):
    try:
        directory = Path('/proc') / str(pid)
        if directory.stat().st_uid != os.getuid():
            return None
        fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
        return int(fields[1]), fields[19]
    except (OSError, ValueError, IndexError):
        return None


def descendants(roots):
    processes = {}
    for directory in Path('/proc').iterdir():
        if directory.name.isdigit():
            pid = int(directory.name)
            info = process_info(pid)
            if info:
                processes[pid] = info
    selected = {pid: processes[pid] for pid in roots if pid in processes}
    while True:
        children = {pid: info for pid, info in processes.items()
                    if info[0] in selected and pid not in selected}
        if not children:
            return selected
        selected.update(children)


def send(pid, info, sig):
    # A PID may have been reused since the process tree was collected.
    current = process_info(pid)
    if current and current[1] == info[1]:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


def stop_session(name):
    target = '=' + name
    session = subprocess.run(['tmux', 'has-session', '-t', target], capture_output=True, text=True)
    if session.returncode:
        message = session.stderr.strip()
        if absent(message):
            print('종료할 ' + name + ' 세션이 없습니다.')
            return
        raise SystemExit(message or 'tmux 관제센터 세션에 접근할 수 없습니다.')
    # list-panes takes a window target; the colon makes the session explicit.
    result = subprocess.run(['tmux', 'list-panes', '-s', '-t', target + ':',
                             '-F', '#{pane_pid} #{pane_dead}'], capture_output=True, text=True)
    if result.returncode:
        message = result.stderr.strip()
        if absent(message) or "can't find window" in message:
            print('종료할 ' + name + ' 세션이 없습니다.')
            return
        raise SystemExit(message or 'tmux 관제센터 세션에 접근할 수 없습니다.')
    roots = [int(line.split()[0]) for line in result.stdout.splitlines()
             if len(line.split()) == 2 and line.split()[1] == '0']
    selected = {}
    try:
        # Freeze supervisors before killing children, so they cannot respawn sensors.
        # Repeat collection to include children forked just before SIGSTOP.
        deadline = time.monotonic() + 10
        while True:
            found = descendants(roots)
            new = {pid: info for pid, info in found.items() if pid not in selected}
            if not new:
                break
            for pid, info in new.items():
                send(pid, info, signal.SIGSTOP)
                selected[pid] = info
            if time.monotonic() >= deadline:
                raise RuntimeError('하위 프로세스 정리가 지연됐습니다. kill 명령을 다시 실행하세요.')
        for pid, info in selected.items():
            send(pid, info, signal.SIGKILL)
    except BaseException:
        for pid, info in selected.items():
            send(pid, info, signal.SIGCONT)
        raise
    result = subprocess.run(['tmux', 'kill-session', '-t', target], capture_output=True, text=True)
    if result.returncode and not absent(result.stderr):
        raise SystemExit(result.stderr.strip())
    print(name + ' 세션과 이 세션에서 시작한 하위 프로세스를 강제 종료했습니다.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', choices=CAN_SESSIONS, default='inha_control_center')
    parser.add_argument('--can-users', action='store_true', help='Stop the three known robot CAN sessions')
    args = parser.parse_args()
    for name in CAN_SESSIONS if args.can_users else (args.session,):
        stop_session(name)


if __name__ == '__main__':
    main()
