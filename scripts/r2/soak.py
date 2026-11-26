"""重复冻结输入，两个后端共用一个进程，逐周期顺序处理，不产生新的定位证据。"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

p = argparse.ArgumentParser()
p.add_argument('--runner',required=True,type=Path)
p.add_argument('--cli',required=True,type=Path)
p.add_argument('--data',required=True,type=Path)
p.add_argument('--out',required=True,type=Path)
p.add_argument('--seconds',type=int,default=600)
a = p.parse_args()
sys.path.insert(0,str(a.runner))
from run import Engine, PROTOCOL

a.out.mkdir()
case = next(c for c in json.loads((a.data/'dataset.json').read_text())['cases'] if c['id']=='static_q2')
started = time.monotonic()
deadline = started
counts = {'icp':0,'gicp':0}
missed = 0
with (a.out/'raw.jsonl').open('x') as rows,(a.out/'cli.stderr').open('x') as log:
    e = Engine(a.cli,a.data,log,2)
    try:
        (a.out/'load.json').write_text(json.dumps(e.load(case)))
        while time.monotonic()-started < a.seconds:
            for backend in counts:
                result=e.match(case,backend,2,PROTOCOL)
                reply=result['replies'][0]
                available=int(next(s for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:')).split()[1])
                temp=max(int(x.read_text())/1000 for x in Path('/sys/class/thermal').glob('thermal_zone*/temp'))
                record=dict(t=time.monotonic()-started,backend=backend,measurement_stamp_ns=case['stamp_ns'],
                    resource=result['after'],fd=len(list(Path('/proc',str(e.p.pid),'fd').iterdir())),temp_c=temp,
                    available_kib=available,accepted=result['accepted'],reason=result['reason'],ms=result['wall_ms'],
                    cache=reply.get('submap'),backend_ms=reply.get('backend_ms'),eval_ms=reply.get('evaluation_ms'))
                rows.write(json.dumps(record)+'\n')
                counts[backend]+=1
                if temp>=85 or available < 1024*1024:
                    raise RuntimeError('controlled load resource guard')
            rows.flush()
            deadline += .5
            now=time.monotonic()
            if deadline < now:
                n=math.ceil((now-deadline)/.5)
                missed+=n
                deadline+=n*.5
            time.sleep(max(0,deadline-time.monotonic()))
    finally:
        e.close()
        (a.out/'end.json').write_text(json.dumps(dict(elapsed_s=time.monotonic()-started,counts=counts,skipped_periods=missed,period_s=.5)))
