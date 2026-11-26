"""Assert the final source/adapter contract against independently received messages."""
import json
import sys
from pathlib import Path


def check(mode, run):
    def read(name):
        return json.loads((run / name).read_text())

    obs = read('observation.json')
    counts = obs['counts']
    assert counts.get('/aft_mapped_to_init', 0) > 1
    assert not any(obs['nonfinite'].values())
    if mode == 'candidate':
        states, scans = '/culvert_source/state', '/culvert_source/scan'
        odom, cloud = '/culvert_test/odom', '/culvert_test/localization_cloud'
        stamps = read('stamps.json')
        times = lambda name: [row[0] for row in stamps[name]]
        assert counts[states] == counts[scans] > 1 and times(states) == times(scans)
        assert len(set(times(states))) == len(times(states))
        assert counts[odom] == counts[cloud] > 0
        assert times(odom) == times(cloud) == times('test_odom -> test_base_link')
        assert set(times(cloud)) <= set(times(states))
        assert counts.get('/culvert_test/imu', 0) > 0
        checks = {'all_xyz_m': 1e-5, 'base_position_m': 1e-8, 'base_rotation_matrix': 1e-8,
                  'tf_position_m': 1e-9, 'tf_rotation_matrix': 1e-9}
        for name, tolerance in checks.items():
            item = obs['compared'][name]
            assert item['count'] == counts[cloud] and item['error']['max'] < tolerance, (name, item)
        source = read('Log/culvert_source_counts.json')
        assert source['published_pairs'] == counts[states]
        assert source['attempts_after_odom'] == counts[states] + sum(source['first_rejection'].values())
        if source['published_trimmed_frames']:
            import csv
            with (run / 'Log/culvert_source_tail.csv').open() as stream:
                trimmed = [r for r in csv.DictReader(stream) if r['decision'] == 'publish_trimmed']
            assert len(trimmed) == source['published_trimmed_frames']
            assert sum(int(r['outside_points']) for r in trimmed) == source['published_trimmed_points']
            assert all(int(r['input_points']) == int(r['inside_points']) + int(r['outside_points'])
                       and float(r['outside_fraction']) <= .01 and float(r['tail_overrun_s']) <= .0005 for r in trimmed)
        assert not any(edge.startswith('map ->') for edges in obs['tf'].values() for edge in edges)
    print('Replay assertions passed:', mode, counts)


if __name__ == '__main__':
    check(sys.argv[1], Path(sys.argv[2]))
