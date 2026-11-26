"""Generate leave-one-frame-out maps in the frozen run's raw FAST-LIVO2 world."""
import argparse
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation

from common import cloud, digest, read_json, save_cloud, transform, write_json


RUNS = {'static': 'candidate_QOvQO5Ya', 'handheld': 'candidate_AbO58Yqm'}


def generate_history(runs, out):
    out.mkdir(parents=True, exist_ok=True)
    cases, manifests = [], {}
    for name, run in RUNS.items():
        folder = runs / run
        p = read_json(folder / 'active_parameters.json')['parameters']
        T_I_L = np.eye(4)
        T_I_L[:3, :3] = np.reshape(p['extrin_calib.extrinsic_R'], (3, 3))
        T_I_L[:3, 3] = p['extrin_calib.extrinsic_T']
        assert np.allclose(T_I_L[:3, :3], np.eye(3)) and np.allclose(T_I_L[:3, 3], [-.011, -.02329, .04412])
        samples = read_json(folder / 'samples/index.json')['samples']
        assert len(samples) == len({s['stamp_ns'] for s in samples}) == 3
        maps, poses = [], []
        for i, s in enumerate(samples):
            state = s['source_state']
            assert state['stamp_ns'] == s['stamp_ns'] and s['frame'] == 'livo_lidar'
            assert s['source_scan_seen_at_stamp'] == 1
            src = folder / 'samples' / s['pcd']
            assert digest(src) == s['sha256']
            xyz = cloud(src)
            assert np.isfinite(xyz).all() and len(xyz) == s['points']
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(state['xyzw']).as_matrix()
            T[:3, 3] = state['xyz_m']
            T = T @ T_I_L
            poses.append(T.tolist())
            maps.append(transform(xyz, T))
            shutil.copyfile(src, out / f'{name}_scan_q{i}.pcd')
        for i, s in enumerate(samples):
            others = [j for j in range(3) if i != j]
            filename = f'{name}_map_q{i}.pcd'
            save_cloud(out / filename, np.vstack([maps[j] for j in others]))
            case = dict(id=f'{name}_q{i}', map=filename, scan=f'{name}_scan_q{i}.pcd',
                        map_sha256=digest(out / filename), map_frame='r2_' + name + '_W',
                        scan_sha256=s['sha256'], scan_id=s['pcd'].split('.')[0], stamp_ns=s['stamp_ns'],
                        source_frame=s['frame'], initial=poses[i], local=poses[i], reference=poses[i], truth=None,
                        candidates=[poses[i]], split='history', scene=name, reference_kind='same_bag_livo',
                        expected=None, map_stamps=[samples[j]['stamp_ns'] for j in others],
                        map_source_sha256=[samples[j]['sha256'] for j in others], map_points=sum(len(maps[j]) for j in others),
                        scan_points=len(maps[i]))
            cases.append(case)
        manifests[name] = dict(run=run, bag=read_json(folder / 'replay.json')['path'],
                               active_parameters_sha256=digest(folder / 'active_parameters.json'),
                               index_sha256=digest(folder / 'samples/index.json'), T_I_L=T_I_L.tolist(),
                               gravity='FAST-LIVO2 W before export gravity rotation', origin='frozen FAST-LIVO2 W',
                               replay=cases[-1])
    write_json(out / 'history.json', dict(cases=cases, runs=manifests))
    return cases


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', type=Path)
    parser.add_argument('out', type=Path)
    a = parser.parse_args()
    generate_history(a.runs, a.out)
