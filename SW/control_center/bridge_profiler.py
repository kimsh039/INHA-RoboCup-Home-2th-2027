"""In-process sampling profiler for the web bridge (ptrace/py-spy are unavailable on the Jetson).

Every 10 ms it records where the main thread is; every 15 s it writes the top call sites to
~/robocup_work/logs/misc/bridge_profile.txt. Disable with INHA_BRIDGE_PROFILE=0.
"""
from collections import Counter
import os
from pathlib import Path
import sys
import threading
import time

OUT = Path(os.environ.get('INHA_BRIDGE_PROFILE_FILE', str(Path.home() / 'robocup_work/logs/misc/bridge_profile.txt')))


def site(frame):
    return f'{Path(frame.f_code.co_filename).name}:{frame.f_code.co_name}:{frame.f_lineno}'


def stack(frame, depth=7):
    out = []
    while frame is not None and len(out) < depth:
        out.append(site(frame))
        frame = frame.f_back
    return ' < '.join(out)


def run(main_ident, interval=0.01, report=15.0):
    leaf, stacks, threads = Counter(), Counter(), Counter()
    names = {}
    samples, started = 0, time.monotonic()
    while True:
        time.sleep(interval)
        frames = sys._current_frames()
        frame = frames.get(main_ident)
        if frame is not None:
            samples += 1
            leaf[site(frame)] += 1
            stacks[stack(frame)] += 1
        for ident, other in frames.items():
            if ident != main_ident and ident != threading.get_ident():
                threads[(ident, site(other))] += 1
        if time.monotonic() - started >= report:
            for t in threading.enumerate():
                names[t.ident] = t.name
            lines = [time.strftime('%H:%M:%S') + f' main-thread samples={samples} (10 ms each, {report:.0f}s window)', '', '[main leaf]']
            lines += [f'{100 * n / max(samples, 1):5.1f}%  {k}' for k, n in leaf.most_common(25)]
            lines += ['', '[main stacks]']
            lines += [f'{100 * n / max(samples, 1):5.1f}%  {k}' for k, n in stacks.most_common(15)]
            lines += ['', '[other threads: name, site, share of samples]']
            lines += [f'{100 * n / max(samples, 1):5.1f}%  {names.get(i, i)}  {k}' for (i, k), n in threads.most_common(20)]
            try:
                OUT.parent.mkdir(parents=True, exist_ok=True)
                OUT.write_text('\n'.join(lines) + '\n')
                with open(OUT.with_suffix('.history'), 'a') as history:
                    history.write('\n'.join(lines[:30]) + '\n\n')
            except OSError:
                pass
            leaf.clear(); stacks.clear(); threads.clear(); samples = 0; started = time.monotonic()


def start():
    if os.environ.get('INHA_BRIDGE_PROFILE', '1') == '0':
        return
    threading.Thread(target=run, args=(threading.main_thread().ident,), name='bridge-profiler', daemon=True).start()
