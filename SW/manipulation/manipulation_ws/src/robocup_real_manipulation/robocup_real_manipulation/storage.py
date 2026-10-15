"""Bound diagnostic disk use independently of durable task replay tombstones."""
from contextlib import contextmanager
import threading

mutex=threading.Lock()
@contextmanager
def append_event(path):
    with mutex:
        if path.exists() and path.stat().st_size>=8*1024*1024:
            for i in range(3,0,-1):
                source=path.with_name(path.name+'.'+str(i))
                if source.exists(): source.replace(path.with_name(path.name+'.'+str(i+1)))
            path.replace(path.with_name(path.name+'.1'))
        with path.open('a') as stream: yield stream
