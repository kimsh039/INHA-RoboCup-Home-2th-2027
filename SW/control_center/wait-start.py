#!/usr/bin/env python3
"""Report actual bridge readiness or its captured startup error to the launcher."""
from pathlib import Path
import subprocess
import sys
import time


def read_log(path):
    try:
        return path.read_text(errors='replace')
    except FileNotFoundError:
        return ''


def main():
    if sys.argv[1] == '--logs':
        files = sorted(Path(sys.argv[2]).glob('*.log'))
        if not files:
            raise SystemExit('저장된 시작 로그가 없습니다. 수정된 실행기로 관제센터를 시작하세요.')
        path = files[-1]
        print(str(path))
        print('\n'.join(read_log(path).splitlines()[-80:]))
        return
    path = Path(sys.argv[1])
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        output = read_log(path)
        session = subprocess.run(['tmux', 'has-session', '-t', '=inha_control_center'],
                                 capture_output=True, text=True)
        if session.returncode:
            print('\n'.join(output.splitlines()[-40:]), file=sys.stderr)
            raise SystemExit(f'관제센터가 시작 도중 종료됐습니다. 시작 로그: {path}')
        ready = next((line for line in output.splitlines() if line.startswith('INHA_CONTROL_CENTER_READY ')), None)
        if ready:
            print('젯슨 관제센터 서버 준비 완료: ' + ready.split(' ', 1)[1])
            return
        time.sleep(0.5)
    print('\n'.join(read_log(path).splitlines()[-40:]), file=sys.stderr)
    raise SystemExit(f'30초 동안 서버 준비 완료 메시지를 받지 못했습니다. 세션이 남아 있을 수 있습니다. 시작 로그: {path}')


if __name__ == '__main__':
    main()
