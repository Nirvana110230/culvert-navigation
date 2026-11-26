"""Create compact, auditable tables and figures from target-host JSONL (requires matplotlib on analysis host)."""
import argparse
import collections
import csv
import json
from pathlib import Path

import numpy as np

from common import cloud, read_json, transform, write_json


def summary(values):
    v = [x for x in values if x is not None and np.isfinite(x)]
    keys = ['p50', 'p95', 'p99', 'max']
    return dict(n=len(v), **dict(zip(keys, np.percentile(v, [50, 95, 99, 100]).tolist() if v else [None]*4)))


def table(path, rows):
    if not rows:
        return
    with path.open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
        writer.writeheader()
        writer.writerows(rows)


def nested(obj, path):
    for key in path.split('.'):
        if not isinstance(obj, dict):
            return None
        obj = obj.get(key)
    return obj


def read_rows(paths, dataset):
    cases = {c['id']: c for c in dataset['cases']}
    rows = []
    freeze = None
    for p in paths:
        frozen = read_json(p.parent / 'freeze.json')
        if freeze is not None and frozen != freeze:
            raise ValueError('Incompatible protocols or binaries')
        freeze = frozen
        for line in p.read_text(encoding='utf-8').splitlines():
            row = json.loads(line)
            c = cases[row['case']]
            reply = row['replies'][row['selected']]
            err = row.get('error')
            good = (err['translation_m'] <= frozen['protocol']['good_translation_m'] and
                    err['rotation_deg'] <= frozen['protocol']['good_rotation_deg']) if err else None
            expected = c['expected']
            valid = bool(expected == 'accept' and good)
            group = ('history' if expected is None else
                     ('correct_accept' if valid else 'false_accept') if row['accepted'] else
                     'correct_pose_rejected' if valid else 'correct_reject' if expected == 'reject' else 'failed_to_localize')
            row.update(split=c['split'], scene=c['scene'], expected=expected, pose_correct=good, outcome=group,
                       reply=reply, bin_m=c.get('bin_m'), bin_deg=c.get('bin_deg'), mode=c.get('mode'),
                       converged=reply.get('converged', False), map_points=c['map_points'], scan_points=c['scan_points'])
            rows.append(row)
    return rows, freeze


def summarized(group, category):
    n = len(group)
    counts = collections.Counter(r['outcome'] for r in group)
    suitable = [r for r in group if r['expected'] == 'accept']
    good = [r for r in suitable if r['pose_correct']]
    wrong = [r for r in group if r['expected'] == 'reject' or (r['expected'] == 'accept' and not r['pose_correct'])]
    def rate(k, denominator):
        return k / denominator if denominator else None
    result = dict(category, n=n, n_expected_accept=len(suitable), n_pose_qualified=len(good), n_unsuitable=len(wrong),
                  accepted=sum(r['accepted'] for r in group), correct_accepted=counts['correct_accept'],
                  correct_rejected=counts['correct_reject'], false_accepted=counts['false_accept'],
                  correct_pose_rejected=counts['correct_pose_rejected'], failed_to_localize=counts['failed_to_localize'],
                  not_converged=sum(not r['converged'] for r in group),
                  rejected=sum(not r['accepted'] for r in group), over500=sum(r['wall_ms'] > 500 for r in group),
                  correct_rate=rate(counts['correct_accept'], len(suitable)),
                  false_accept_among_unsuitable=rate(counts['false_accept'], len(wrong)),
                  correct_pose_reject_rate=rate(counts['correct_pose_rejected'], len(good)),
                  not_converged_rate=rate(sum(not r['converged'] for r in group), n),
                  reasons=dict(collections.Counter(r['reason'] for r in group)))
    for k in ['rmse', 'weak_ratio', 'inlier_ratio', 'backend_ms', 'evaluation_ms', 'elapsed_ms']:
        result[k] = summary(r['reply'].get(k) for r in group)
    result['wall_ms'] = summary(r['wall_ms'] for r in group)
    for k in ['translation_m', 'rotation_deg', 'long_m', 'lat_m', 'vert_m', 'roll_deg', 'pitch_deg', 'yaw_deg']:
        result['abs_' + k] = summary(abs(r['error'][k]) for r in group if r['error'])
        result['accepted_abs_' + k] = summary(abs(r['error'][k]) for r in group if r['error'] and r['accepted'])
    return result


def group_rows(rows, keys):
    groups = collections.defaultdict(list)
    for r in rows:
        groups[tuple(r[k] for k in keys)].append(r)
    return [(dict(zip(keys, key)), group) for key, group in groups.items()]


def flatten(row):
    out = {}
    for k, v in row.items():
        if isinstance(v, dict) and set(v) >= {'p50', 'max', 'n'}:
            out.update({k + '_' + sub: val for sub, val in v.items()})
        else:
            out[k] = json.dumps(v, ensure_ascii=False) if isinstance(v, (dict, list)) else v
    return out


def perf(rows):
    output = []
    for r in rows:
        r['cache'] = 'cold' if r['rep'] == 0 else 'hot'
    for group, rs in group_rows(rows, ['case', 'map_points', 'backend', 'threads', 'cache']):
        r = dict(group, n=len(rs), threads_effective=rs[0]['reply'].get('threads_effective'),
                 os_threads_seen=sorted(set(x['after']['os_threads'] for x in rs)),
                 accepted=sum(x['accepted'] for x in rs), over500=sum(x['wall_ms'] > 500 for x in rs))
        r['wall_ms'] = summary(x['wall_ms'] for x in rs)
        r['load_ms'] = summary(x['load']['wall_ms'] for x in rs if x['rep'] == 0)
        r['first_result_including_load_ms'] = summary(x['load']['wall_ms']+x['wall_ms'] for x in rs if x['rep']==0)
        r['rss_mib'] = summary(x['after']['rss_kib']/1024 for x in rs)
        r['peak_rss_mib'] = max(x['after']['hwm_kib']/1024 for x in rs)
        r['cpu_pct_one_core'] = summary(100*x['reply']['cpu_ms']/x['wall_ms'] for x in rs)
        for k in ['scan_read_ms', 'submap.lookup_ms', 'backend_ms', 'evaluation_ms', 'elapsed_ms',
                  'stages.gicp.source_preprocess_ms', 'stages.gicp.target_preprocess_ms', 'stages.gicp.align_ms',
                  'stages.rough.elapsed_ms', 'stages.fine.elapsed_ms', 'submap.fine_points']:
            r[k] = summary(nested(x['reply'], k) for x in rs)
        output.append(r)
    return output


def figures(rows, data, cases, out, performance):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    available={f.name for f in font_manager.fontManager.ttflist}
    family=next((name for name in ['Microsoft YaHei','Noto Sans CJK SC','SimHei'] if name in available), None)
    if family:
        plt.rcParams['font.family']=[family]
    group = [r for r in rows if r['split'] == 'holdout' and r['mode'] == 'combined']
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6), constrained_layout=True)
    for backend in ['icp', 'gicp']:
        sub = [r for r in group if r['backend'] == backend]
        mags = sorted(set(r['bin_m'] for r in sub))
        fractions = []
        for m in mags:
            selected = [r for r in sub if r['bin_m'] == m]
            fractions.append(sum(r['outcome']=='correct_accept' for r in selected)/len(selected))
        ax[0].plot(mags, fractions, 'o-', label=backend.upper())
        measurements = [p for p in performance if p['backend'] == backend and p['cache']=='hot' and p['threads']==1]
        ax[1].plot([p['map_points'] for p in measurements], [p['wall_ms']['p95'] for p in measurements],
                   'o-', label=backend.upper())
    ax[0].set(xlabel='平移与旋转同时扰动（m / °）', ylabel='正确放行比例', ylim=(-.07,1.07))
    ax[0].set_xticks([.1,.3,.5,1,2],['0.1/2','0.3/5','0.5/10','1/20','2/40'])
    ax[1].set(xlabel='地图原始点数', ylabel='单线程热缓存总时延 P95（ms）')
    for a in ax:
        a.grid(alpha=.25)
        a.legend()
    fig.savefig(out / 'accuracy_and_time.png', dpi=175)
    plt.close(fig)
    # Same coordinate scales within each panel. Ambiguous alternative pose is shown even when blocked.
    picks = [(next((r for r in rows if r['case']==case and r['backend']==backend), None), title)
             for case, backend, title in [('holdout_mixed', 'gicp', '六自由度已知真值'),
                                          ('holdout_walls_3.0', 'icp', '直廊纵向退化'),
                                          ('holdout_repeat_0.018', 'gicp', '近似重复段'),
                                          ('handheld_q2', 'gicp', '同袋手持留一查询')]]
    failure = max((r for r in rows if r['split']=='holdout' and r['outcome']=='false_accept'),
                  key=lambda r: r['error']['translation_m'], default=None)
    if failure:
        picks.append((failure, '留出集错误放行'))
    fig, axes = plt.subplots(len(picks), 2, figsize=(9, 2.4*len(picks)), squeeze=False, constrained_layout=True)
    for (r, title), pair in zip(picks, axes):
        if not r:
            continue
        c = cases[r['case']]
        target, scan = cloud(data / c['map']), cloud(data / c['scan'])
        for axis_no, a in enumerate(pair, start=1):
            tar = target[::max(1, len(target)//12000)]
            a.scatter(tar[:,0], tar[:,axis_no], s=.5, c='0.72', label='地图', rasterized=True)
            for i, rep in enumerate(r['replies']):
                if rep.get('T_target_source') is not None:
                    pts = transform(scan, rep['T_target_source'])
                    a.scatter(pts[::3,0], pts[::3,axis_no], s=.8, alpha=.6, label=f'候选 {i+1}')
            if r['outcome']=='false_accept':
                for transform_name,T,marker,color in [('真值',c['truth'],'*','#252222'),
                                                       ('估计',r['reply']['T_target_source'],'X','#bc3a24')]:
                    pos=np.asarray(T)[:3,3]
                    a.scatter(pos[0],pos[axis_no],marker=marker,c=color,s=45,zorder=5,label=transform_name)
            a.set_aspect('equal', adjustable='datalim')
            reason={'accepted':'放行','weak_geometry':'弱约束拒绝',
                    'ambiguous_multiple_candidates':'多解拒绝'}.get(r['reason'],r['reason'])
            a.set(xlabel='地图 X (m)', ylabel=['', 'Y (m)', 'Z (m)'][axis_no],
                  title=f'{title} · {reason}')
            a.legend(fontsize=7, loc='upper right', markerscale=3)
    fig.savefig(out / 'registration_overlays.png', dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('raw', nargs='+', type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    dataset = read_json(args.data / 'dataset.json')
    cases = {c['id']: c for c in dataset['cases']}
    rows, frozen = read_rows(args.raw, dataset)
    accuracy = [r for r in rows if r['kind']=='accuracy']
    counts = collections.Counter((r['case'],r['backend']) for r in accuracy)
    if max(counts.values(), default=1) > 1:
        raise ValueError('duplicate accuracy case')
    groups = []
    for keys in [['split', 'backend'], ['split', 'scene', 'backend'], ['split','mode','bin_m','bin_deg','backend']]:
        selected = [r for r in accuracy if 'mode' not in keys or r['scene']=='initial_bias']
        groups.extend(summarized(v,k) for k,v in group_rows(selected,keys))
    performance = perf([r for r in rows if r['kind']=='performance'])
    write_json(args.out / 'summary.json', dict(freeze=frozen, groups=groups, performance=performance))
    table(args.out / 'accuracy_groups.csv', [flatten(x) for x in groups])
    table(args.out / 'performance.csv', [flatten(x) for x in performance])
    table(args.out / 'cases.csv', [dict(case=r['case'], backend=r['backend'], split=r['split'], scene=r['scene'],
                                         accepted=r['accepted'], converged=r['converged'], reason=r['reason'],
                                         outcome=r['outcome'], rmse_m=r['reply'].get('rmse'), overlap=r['reply'].get('inlier_ratio'),
                                         weak_ratio=r['reply'].get('weak_ratio'), wall_ms=r['wall_ms'],
                                         n_candidates=len(r['replies']), **(r['error'] or {}),
                                         **{'internal_' + k: v for k,v in (r['reference_delta'] or {}).items()}) for r in accuracy])
    table(args.out / 'evictions.csv', [dict(case=r['case'], backend=r['backend'], threads=r['threads'],
                                            step=r['step'], **r['reply'].get('submap',{})) for r in rows if r['kind']=='eviction'])
    figures(accuracy, args.data, cases, args.out, performance)
    print(f'{len(accuracy)} accuracy, {len(performance)} performance groups')


if __name__ == '__main__':
    main()
