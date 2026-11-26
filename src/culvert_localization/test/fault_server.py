#!/usr/bin/env python3
"""Controlled JSONL worker used only by the ROS fault-injection test."""
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

generation = 0
root = Path(os.environ['CULVERT_FAULT_DIR'])
for line in sys.stdin:
    req = json.loads(line)
    if req['op'] == 'load':
        if hashlib.sha256(Path(req['path']).read_bytes()).hexdigest() == req['sha256']:
            generation += 1
            meta = req
            reply = dict(ok=True, generation=generation, map=meta)
        else:
            reply = dict(ok=False, reason='map_sha256_mismatch')
    else:
        fault = json.loads((root/'fault.json').read_text())
        if fault.get('crash'):
            sys.exit(17)
        time.sleep(fault.get('delay_s', 0))
        pose = np.array(req['initial'])
        correction = pose @ np.linalg.inv(req['local'])
        reply = dict(ok=True, accepted=True, reason='accepted', converged=True,
                     generation=generation, map_id=meta['id'], map_version=meta['version'],
                     map_sha256=meta['sha256'], target_frame=meta['frame'],
                     source_frame=req['source_frame'], stamp_ns=req['stamp_ns'],
                     scan_id=req['scan_id'], backend=req['backend'],
                     T_target_source=pose.tolist(), T_map_W=correction.tolist(),
                     rmse=0., inlier_ratio=1., weak_ratio=.1, elapsed_ms=.2)
        reply.update(fault.get('override',{}))
    print(json.dumps(reply), flush=True)
