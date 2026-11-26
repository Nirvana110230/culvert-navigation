"""按进程采样，包含定位节点启动的 C++ 子进程。总 CPU 占用另由 /proc/stat 差分。"""
import json
import os
import sys
import time
from pathlib import Path

pids = {name:int(pid) for name,pid in (v.split('=') for v in sys.argv[1:])}
tick = os.sysconf('SC_CLK_TCK')
page_kib = os.sysconf('SC_PAGE_SIZE') // 1024
start = time.monotonic()
def sample(pid):
    folder = Path('/proc') / str(pid)
    fields = (folder/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,cpu_s=(int(fields[11])+int(fields[12]))/tick,
                threads=int(fields[17]),rss_kib=int(fields[21])*page_kib,fd=len(list((folder/'fd').iterdir())))
while Path('/proc',str(pids['mapper'])).exists():
    procs = {}
    for name,pid in pids.items():
        try:
            procs[name] = sample(pid)
            children = set()
            for task in Path(f'/proc/{pid}/task').glob('*/children'):
                try:
                    children.update(task.read_text().split())
                except (FileNotFoundError, ProcessLookupError, PermissionError):
                    continue
            for i,child in enumerate(sorted(children)):
                procs[f'{name}_child{i}'] = sample(int(child))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            pass
    stat = list(map(int,Path('/proc/stat').read_text().splitlines()[0].split()[1:9]))
    avail = int(next(s for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:')).split()[1])
    temperatures=[int(p.read_text())/1000 for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')]
    print(json.dumps(dict(wall_s=time.monotonic()-start, processes=procs,cpu_ticks=sum(stat),
          idle_ticks=stat[3]+stat[4],available_kib=avail,load1=os.getloadavg()[0],temp_c=max(temperatures))),flush=True)
    time.sleep(.5)
