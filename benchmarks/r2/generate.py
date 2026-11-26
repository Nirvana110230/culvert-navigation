"""Generate seeded, split-by-scene benchmarks, leave-one-out maps and map scales."""
import argparse
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from common import cloud, digest, pose, read_json, save_cloud, transform, write_json
from history import generate_history


SEEDS = {'debug': 41713, 'tune': 68023, 'holdout': 93487}
BINS = [(0, 0), (.1, 2), (.3, 5), (.5, 10), (1, 20), (2, 40)]


def surfaces(rng, geometry, n, extent=(-11, 11), structures=True):
    """Continuous surfaces independently sampled in map and scan, with no longitudinal end plates."""
    width, height, features = geometry
    x = rng.uniform(*extent, n)
    y = rng.uniform(-width, width, n)
    z = rng.uniform(0, height, n)
    face = rng.integers(0, 4, n)
    y[face == 0] = -width
    y[face == 1] = width
    z[face == 2] = 0
    z[face == 3] = height
    parts = [np.c_[x, y, z]]
    for center, sides in features if structures else []:
        # Small asymmetric structural blocks with independent, area-distributed visible faces.
        pts = rng.uniform(-.5, .5, (n // 10, 3)) * sides
        which = rng.integers(0, 3, len(pts))
        pts[np.arange(len(pts)), which] = rng.choice([-.5, .5], len(pts)) * sides[which]
        pts += center
        parts.append(pts[(pts[:, 0] >= extent[0]) & (pts[:, 0] <= extent[1])])
    return np.vstack(parts)


def geometry(rng):
    w, h = rng.uniform(1.7, 2.3), rng.uniform(2.7, 3.4)
    centers = [[rng.uniform(-3, -2), -w + .45, .9], [rng.uniform(-.5, .3), w - .4, 1.7],
               [rng.uniform(1.5, 2.5), -.55, .5], [rng.uniform(3, 4), w - .6, 2.1]]
    return w, h, [(np.array(c), rng.uniform([.45, .55, .8], [1.1, .9, 1.4])) for c in centers]


class Dataset:
    def __init__(self, out):
        self.out = out
        self.cases = []
        self.assets = {}

    def save(self, name, xyz):
        file = name + '.pcd'
        save_cloud(self.out / file, xyz)
        self.assets[file] = (digest(self.out / file), len(xyz))
        return file

    def case(self, name, target, scan, truth, initial, split, scene, expected, **extra):
        map_sha, map_n = self.assets[target]
        scan_sha, scan_n = self.assets[scan]
        initial = initial.tolist()
        self.cases.append(dict(id=name, map=target, scan=scan, map_sha256=map_sha, map_points=map_n,
                               scan_sha256=scan_sha, scan_points=scan_n, stamp_ns=10**18 + len(self.cases) * 10**8,
                               scan_id=name, source_frame='r2_lidar', map_frame='r2_map', initial=initial,
                               local=pose().tolist(), reference=None, truth=None if truth is None else truth.tolist(),
                               candidates=extra.pop('candidates', [initial]), split=split, scene=scene,
                               expected=expected, **extra))


def add_split(ds, split, seed):
    rng = np.random.default_rng(seed)
    world = geometry(rng)
    full = surfaces(rng, world, 6500)
    target = ds.save(f'{split}_asymmetric_map', full)
    designs = [('identity', pose()), ('translation', pose([.85, -.32, .54])),
               ('rotation', pose(angles=[13, -8, 27])),
               ('mixed', pose([.7, -.36, .88], [11, -8, 31])),
               ('reverse', pose([.4, .12, 1], [3, -2, 177]))]
    for number, (name, T) in enumerate(designs):
        pts = surfaces(rng, world, 2400, (-4.8, 4.8))
        src = ds.save(f'{split}_{name}', transform(pts, np.linalg.inv(T)))
        ds.case(f'{split}_{name}', target, src, T, T, split, name, 'accept', bin_m=0, bin_deg=0, mode='exact_prior')
    T = designs[3][1]
    # Separate pure translation, pure rotation and simultaneous offsets using the same scan across backends.
    for rep in range(3):
        pts = surfaces(rng, world, 2500, (-4.8, 4.8))
        scan = ds.save(f'{split}_bias_{rep}', transform(pts, np.linalg.inv(T)) + rng.normal(0, .006, pts.shape))
        a = rng.normal(size=3)
        a /= np.linalg.norm(a)
        b = rng.normal(size=3)
        b /= np.linalg.norm(b)
        for m, deg in BINS[1:]:
            for mode, dist, angle in [('translation', m, 0), ('rotation', 0, deg), ('combined', m, deg)]:
                prior = T.copy()
                prior[:3, 3] += dist * a
                prior[:3, :3] = Rotation.from_rotvec(np.deg2rad(angle) * b).as_matrix() @ T[:3, :3]
                ds.case(f'{split}_{mode}_{m}_{deg}_{rep}', target, scan, T, prior,
                        split, 'initial_bias', 'accept', bin_m=dist, bin_deg=angle, mode=mode)
    scan_xyz = transform(surfaces(rng, world, 2600, (-4.8, 4.8)), np.linalg.inv(T))
    perturbed = T.copy()
    perturbed[:3, 3] += [.12, -.04, .03]
    for name, xyz in [('noise', scan_xyz + rng.normal(0, .045, scan_xyz.shape)),
                      ('dynamic', np.vstack([scan_xyz, rng.normal([1, .4, .5], [.5, .25, .7], (650, 3))])),
                      ('occlusion', scan_xyz[(scan_xyz[:, 0] > -.8) & (scan_xyz[:, 1] < 1.4)]),
                      ('sparse', scan_xyz[:9]), ('empty', scan_xyz[:0]),
                      ('nonfinite', np.full((48, 3), np.nan))]:
        scan = ds.save(f'{split}_{name}', xyz)
        ds.case(f'{split}_{name}', target, scan, T, perturbed, split, name,
                'reject' if name in {'sparse', 'empty', 'nonfinite'} else 'accept')
    # Infinite-axis proxy: independently sampled x, map endpoints >=12 m outside all query extents.
    long_map = ds.save(f'{split}_walls_map', surfaces(rng, world, 16500, (-22, 22), False))
    pts = surfaces(rng, world, 3600, (-4, 4), False)
    scan = ds.save(f'{split}_walls', transform(pts, np.linalg.inv(T)))
    for shift in [0, 1., 3.]:
        prior = T.copy()
        prior[0, 3] += shift
        ds.case(f'{split}_walls_{shift}', long_map, scan, T, prior, split, 'parallel_walls', 'reject',
                longitudinal_prior_shift=shift)
    # Two separated entrances. Supply both hypotheses from a fixed surveyed translation for exact/near repeats.
    period = 24.
    for variation in (0., .018):
        clone = full.copy() + [period, 0, 0]
        clone[:, 1] += variation * np.sin(clone[:, 0])
        repeated = ds.save(f'{split}_repeat_map_{variation}', np.vstack([full, clone]))
        scan = ds.save(f'{split}_repeat_scan_{variation}', scan_xyz)
        other = perturbed.copy()
        other[0, 3] += period
        ds.case(f'{split}_repeat_{variation}', repeated, scan, T, perturbed, split,
                'repeat_exact' if variation == 0 else 'repeat_near', 'reject',
                candidates=[other.tolist(), perturbed.tolist()])
    # Deliberately mismatched map. Sensor scan stays identical. No valid pose on this map.
    invalid_map = ds.save(f'{split}_unrelated_map', rng.uniform(-.5, .5, (6000, 3)) + [5., 5., 4.])
    scan = ds.save(f'{split}_wrong_scan', scan_xyz)
    ds.case(f'{split}_wrong_map', invalid_map, scan, None, perturbed, split, 'wrong_map', 'reject')
    # Reuse this unmodified, independent scan of the asymmetric corridor in every scale test.
    return full, target, f'{split}_mixed.pcd', T


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--r1-runs', required=True, type=Path)
    parser.add_argument('--r1-synthetic', required=True, type=Path)
    parser.add_argument('--out', type=Path, required=True)
    a = parser.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    ds = Dataset(a.out)
    for split, seed in SEEDS.items():
        full, target, scan, T = add_split(ds, split, seed)
    for copies in [1, 5, 16]:
        tiled = [full + [32. * i, 0, 0] for i in range(copies)]
        name = ds.save(f'scale_{copies}', np.vstack(tiled))
        ds.case(f'scale_{copies}', name, scan, T, T, 'scale', 'map_scale', 'accept', tiles=copies)
    r1 = read_json(a.r1_synthetic / 'truth.json')
    for fname, sha in r1['sha256'].items():
        src = a.r1_synthetic / fname
        assert digest(src) == sha
        shutil.copyfile(src, a.out / ('r1_' + fname))
        ds.assets['r1_' + fname] = (sha, len(cloud(src)))
    T = np.array(r1['T_map_d_lidar_d'])
    for name, delta in [('exact', pose()), ('biased', pose([.11, -.03, .04], [1., -1., 2.]))]:
        ds.case('r1_' + name, 'r1_map.pcd', 'r1_scan.pcd', T, delta @ T, 'r1', 'round1', 'accept')
    ds.cases.extend(generate_history(a.r1_runs, a.out))
    write_json(a.out / 'dataset.json', dict(version='r2-fixture-v1', seeds=SEEDS, bins=BINS, cases=ds.cases))
    print(f'{len(ds.cases)} cases, dataset SHA256 {digest(a.out / "dataset.json")}')


if __name__ == '__main__':
    main()
