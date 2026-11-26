import json
import math
import os
from pathlib import Path
import re
import select
import subprocess
import threading
import time

import numpy as np


def se3(values):
    m = np.asarray(values, dtype=float)
    if (m.shape != (4, 4) or not np.isfinite(m).all() or
            not np.allclose(m[3], [0, 0, 0, 1], atol=1e-6, rtol=0) or
            not np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=1e-5, rtol=0) or
            abs(np.linalg.det(m[:3, :3])-1) > 1e-5):
        raise ValueError('invalid_SE3')
    return m


def pose_matrix(p):
    x, y, z, w = (getattr(p.orientation, k) for k in 'xyzw')
    if not math.isclose(x*x+y*y+z*z+w*w, 1., rel_tol=0., abs_tol=1e-5):
        raise ValueError('invalid_quaternion')
    m = np.eye(4)
    m[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    m[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return se3(m)


def assign_transform(m, position, q):
    position.x, position.y, position.z = map(float, m[:3, 3])
    r = m[:3, :3]
    # Davenport's symmetric matrix is stable at rotations of 180 degrees as well.
    skew = np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])
    k = np.empty((4, 4))
    k[:3, :3] = r + r.T - np.eye(3)*np.trace(r)
    k[:3, 3] = k[3, :3] = skew
    k[3, 3] = np.trace(r)
    quat = np.linalg.eigh(k)[1][:, -1]
    q.x, q.y, q.z, q.w = map(float, quat)


def cloud_xyz(msg):
    count = msg.width * msg.height
    if (not 0 < count <= 300000 or msg.is_bigendian or msg.point_step <= 0 or
            msg.width*msg.point_step > msg.row_step or
            msg.height*msg.row_step != len(msg.data) or len(msg.data) > 8*1024*1024):
        raise ValueError('invalid_cloud_layout')
    offsets = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z', 'intensity'):
            if (f.name in offsets or f.count != 1 or f.datatype != 7 or
                    f.offset + 4 > msg.point_step or
                    any(abs(f.offset-o) < 4 for o in offsets.values())):
                raise ValueError('invalid_cloud_field')
            offsets[f.name] = f.offset
    if not all(key in offsets for key in 'xyz'):
        raise ValueError('missing_xyz')
    views = {k: np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data, offset=o,
                           strides=(msg.row_step, msg.point_step)) for k, o in offsets.items()}
    if not all(np.isfinite(a).all() for a in views.values()):
        raise ValueError('nonfinite_cloud')
    return np.stack([views[k] for k in 'xyz'], axis=-1).reshape(count, 3)


def map_request(filename):
    if not filename:
        raise ValueError('map_not_configured')
    file = Path(filename)
    if file.stat().st_size > 65536:
        raise ValueError('map_manifest_oversize')
    meta = json.loads(file.read_text(encoding='utf-8'))
    keys = ('path', 'id', 'version', 'sha256', 'frame', 'units', 'gravity', 'origin')
    if (not isinstance(meta, dict) or any(not isinstance(meta.get(k), str) or not meta[k].strip()
            for k in keys) or not re.fullmatch('[a-fA-F0-9]{64}', meta['sha256']) or
            meta['units'] != 'm' or not re.fullmatch('[A-Za-z][A-Za-z0-9_/]*', meta['frame'])):
        raise ValueError('invalid_map_manifest')
    req = {k: meta[k] for k in keys}
    req.update(op='load', path=str((file.parent / meta['path']).resolve()))
    return req


class Server:
    """One worker owns the persistent process and all request/reply I/O."""

    def __init__(self, binary):
        self.binary = binary
        self.process = None
        self.cancelled = threading.Event()

    def close(self):
        p, self.process = self.process, None
        if p is not None:
            p.stdin.close()
            p.stdin = None
            try:
                p.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                p.terminate()
                try:
                    p.communicate(timeout=2)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.communicate()

    def transact(self, request):
        if request['op'] == 'load' and (self.process is None or self.process.poll() is not None):
            self.close()
            self.process = subprocess.Popen([self.binary], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, bufsize=0)
            os.set_blocking(self.process.stdin.fileno(), False)
            os.set_blocking(self.process.stdout.fileno(), False)
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('worker_unavailable')
        p = self.process
        def ready(fd, write):
            while not self.cancelled.is_set():
                reads, writes, _ = select.select([] if write else [fd], [fd] if write else [],
                                                 [], .1)
                if reads or writes:
                    return
            raise RuntimeError('worker_stopping')
        try:
            payload = memoryview((json.dumps(request, allow_nan=False, separators=(',', ':'))+'\n').encode())
            while payload:
                fd = p.stdin.fileno()
                ready(fd, True)
                payload = payload[os.write(fd, payload[:65536]):]
            line = bytearray()
            while not line.endswith(b'\n'):
                fd = p.stdout.fileno()
                ready(fd, False)
                chunk = os.read(fd, 65536)
                if not chunk or len(line) + len(chunk) > 1024*1024:
                    raise RuntimeError('worker_protocol_error')
                line.extend(chunk)
            reply = json.loads(line)
            if not isinstance(reply, dict):
                raise ValueError('worker_reply_type')
            json.dumps(reply, allow_nan=False)
            return reply
        except Exception:
            self.close()
            raise
