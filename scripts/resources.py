import csv
import os
from pathlib import Path
import sys
import time

interval = os.sysconf('SC_CLK_TCK')
start = time.monotonic()
writer = csv.writer(sys.stdout)
writer.writerow(['seconds','mapper_cpu_s','mapper_rss_kib','adapter_cpu_s','adapter_rss_kib','max_thermal_zone_c'])
while True:
    values=[]
    for pid in map(int,sys.argv[1:]):
        try:
            proc=Path('/proc')/str(pid)
            stat=(proc/'stat').read_text().rsplit(')',1)[1].split()
            values.extend([(int(stat[11])+int(stat[12]))/interval,int(stat[21])*os.sysconf('SC_PAGE_SIZE')//1024])
        except FileNotFoundError:
            values.extend([None,None])
    if values[0] is None:
        break
    temps=[int(p.read_text())/1000 for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')]
    writer.writerow([round(time.monotonic()-start,3),*values,max(temps,default=None)])
    sys.stdout.flush()
    time.sleep(.5)
