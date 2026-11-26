"""Verify every named scan/map against frozen manifest, including history disjointness."""
import argparse
from collections import Counter
from pathlib import Path

import numpy as np

from common import cloud, digest, read_json, transform, write_json


def check(folder):
    cases = read_json(folder / 'dataset.json')['cases']
    assert len(cases) == len({c['id'] for c in cases})
    assets = {}
    for c in cases:
        for key in ['map', 'scan']:
            name = c[key]
            expected = c[key + '_sha256'], c[key + '_points']
            if name in assets:
                assert assets[name] == expected
                continue
            file = folder / name
            actual = digest(file), len(cloud(file))
            if actual != expected:
                raise ValueError(f'{name}: expected {expected}, got {actual}')
            assets[name] = actual
    history = sorted((c for c in cases if c['split']=='history'), key=lambda c: c['id'])
    for c in history:
        contributors = [q for q in history if q['scene']==c['scene'] and q['id']!=c['id']]
        assert len(contributors) == 2 and c['stamp_ns'] not in c['map_stamps']
        assert [q['stamp_ns'] for q in contributors] == c['map_stamps']
        assert [q['scan_sha256'] for q in contributors] == c['map_source_sha256']
        expected = np.vstack([transform(cloud(folder / q['scan']), q['local']) for q in contributors])
        np.testing.assert_allclose(cloud(folder / c['map']), expected, atol=3.e-6, rtol=2.e-7)
    return dict(dataset_sha256=digest(folder / 'dataset.json'), split_counts=dict(Counter(c['split'] for c in cases)),
                distinct_clouds_verified=len(assets), history_exclusions_verified=len(history))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('data', type=Path)
    parser.add_argument('out', type=Path)
    args = parser.parse_args()
    result = check(args.data)
    write_json(args.out, result)
    print(result)
