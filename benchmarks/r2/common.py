"""PCD XYZ/XYZI and rigid transforms used by the frozen offline experiments."""
import hashlib
import json
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, obj):
    Path(path).write_text(json.dumps(obj, indent=2, allow_nan=False, ensure_ascii=False) + '\n', encoding='utf-8')


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def cloud(path):
    header, payload = Path(path).read_bytes().split(b'DATA binary\n', 1)
    fields = dict(line.split(maxsplit=1) for line in header.splitlines())
    names = fields[b'FIELDS'].split()
    if names not in ([b'x', b'y', b'z'], [b'x', b'y', b'z', b'intensity']):
        raise ValueError(f'unsupported binary fields {names}')
    assert fields[b'TYPE'].split() == [b'F'] * len(names)
    assert fields[b'SIZE'].split() == [b'4'] * len(names)
    xyz = np.frombuffer(payload, dtype='<f4').reshape(int(fields[b'POINTS']), len(names))[:, :3]
    return xyz.astype(float)


def save_cloud(path, xyz):
    points = np.asarray(xyz, dtype='<f4').reshape(-1, 3)
    header = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
    Path(path).write_bytes(header.encode('ascii') + points.tobytes())


def transform(xyz, T):
    T = np.asarray(T)
    return np.asarray(xyz) @ T[:3, :3].T + T[:3, 3]


def pose(position=(0, 0, 0), angles=(0, 0, 0)):
    T = np.eye(4)
    T[:3, 3] = position
    x, y, z = np.deg2rad(angles)
    sx, sy, sz = np.sin([x, y, z])
    cx, cy, cz = np.cos([x, y, z])
    T[:3, :3] = [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                 [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]]
    return T


def error(estimate, reference):
    E, R = np.asarray(estimate), np.asarray(reference)
    d = E[:3, 3] - R[:3, 3]
    # Left rotation residual is expressed about map axes. XYZ Euler angles in degrees.
    rot = E[:3, :3] @ R[:3, :3].T
    # Polar projection removes round-off from a serialized float32 rotation.
    u, _, v = np.linalg.svd(rot)
    rot = u @ np.diag([1, 1, np.linalg.det(u @ v)]) @ v
    pitch = np.arcsin(np.clip(-rot[2, 0], -1, 1))
    if abs(np.cos(pitch)) > 1e-8:
        roll, yaw = np.arctan2(rot[2, 1], rot[2, 2]), np.arctan2(rot[1, 0], rot[0, 0])
    else:
        roll, yaw = np.arctan2(-rot[1, 2], rot[1, 1]), 0.
    angle = np.arccos(np.clip((np.trace(rot) - 1) / 2, -1, 1))
    return dict(zip(('long_m', 'lat_m', 'vert_m'), d.tolist()),
                **dict(zip(('roll_deg', 'pitch_deg', 'yaw_deg'), np.rad2deg([roll, pitch, yaw]).tolist())),
                translation_m=float(np.linalg.norm(d)), rotation_deg=float(np.rad2deg(angle)))
