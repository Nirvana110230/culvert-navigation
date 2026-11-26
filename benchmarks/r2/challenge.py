"""Additional held-out negative controls, generated and measured with NumPy on RK3588."""
import argparse
from pathlib import Path

import numpy as np

from common import digest, error, pose, read_json, save_cloud, transform, write_json
from run import Engine, PROTOCOL, select_candidate, write_row


SEED = 12013411


def passage(rng, n, span, width, height, seams=False, blocks=False):
    x = rng.uniform(-span, span, n)
    y = rng.uniform(-width/2, width/2, n)
    z = rng.uniform(0, height, n)
    sides = rng.integers(0, 4, n)
    y[sides == 0] = -width/2
    y[sides == 1] = width/2
    z[sides == 2] = 0
    z[sides == 3] = height
    parts = [np.c_[x,y,z]]
    if seams:
        # Paired inward flanges with periodic transverse faces, repeated every 1.2 m.
        for k in range(-int(span/1.2), int(span/1.2)+1):
            m = int(n/span*.12)
            sign = rng.choice([-1,1], m)
            sx = k*1.2 + rng.choice([-.06,.06], m)
            sy = sign*(width/2-rng.uniform(0, .14, m))
            parts.append(np.c_[sx,sy,rng.uniform(0,height,m)])
    if blocks:
        for center, extent in [([-2.2,-1.1,.8],[1,.7,1.2]), ([1.7,.9,1.3],[.65,.8,1.7]),
                               ([3.,-.4,.38],[1.2,.7,.75])]:
            half = np.array(extent)/2
            xyz = rng.uniform(-1,1,(n//9,3))*half
            axis = rng.integers(0,3,len(xyz))
            xyz[np.arange(len(xyz)),axis] = rng.choice([-1,1],len(xyz))*half[axis]
            parts.append(xyz + center)
    return np.vstack(parts)


def generate(out):
    out.mkdir()
    rng = np.random.default_rng(SEED)
    T = pose([.2,-.04,.68],[4,-3,9])
    maps, cases = {}, []
    def save(name, xyz):
        file = out / (name+'.pcd')
        save_cloud(file, xyz)
        return file.name, digest(file), len(xyz)
    def target(name, xyz):
        maps[name] = save(name, xyz)
        return name
    def case(name, scene, m, scan, priors, expected, **extra):
        file, sha, count = maps[m]
        cases.append(dict(id=name, map=file, map_sha256=sha, map_points=count, map_frame='r2_supp_map',
                          scan=scan[0], scan_sha256=scan[1], scan_points=scan[2],
                          source_frame='r2_supp_lidar', scan_id=scan[1][:16], stamp_ns=10**18+100*len(cases),
                          initial=priors[0].tolist(), candidates=[p.tolist() for p in priors],
                          local=np.eye(4).tolist(), reference=None, truth=T.tolist(), split='supplemental',
                          scene=scene, expected=expected, **extra))
    plane = target('long_walls', passage(rng, 20000, 24, 3.4, 2.8))
    long_points = passage(rng, 3200, 3.8, 3.4, 2.8)
    long_scan = save('long_query', transform(long_points, np.linalg.inv(T)))
    for axis, offsets in [('x',[-3,-1,-.5,0,.5,1,3]), ('y',[-.5,-.1,.1,.5]), ('yaw',[-15,-5,5,15])]:
        for offset in offsets:
            prior = T.copy()
            if axis=='yaw':
                prior[:3,:3] = pose(angles=(0,0,offset))[:3,:3] @ T[:3,:3]
            else:
                prior[0 if axis=='x' else 1,3] += offset
            case(f'walls_{axis}_{offset}', 'longitudinal_nullspace', plane, long_scan, [prior], 'reject',
                 perturbed_axis=axis, bias=offset)
    narrow = target('narrow_080_seams',passage(rng, 16500, 18, .8, 1.9, seams=True))
    narrow_scan = save('narrow_query',transform(passage(rng, 3400, 4.6, .8, 1.9, seams=True), np.linalg.inv(T)))
    seed_poses = []
    for phase in [0,1.2,2.4]:
        prior = T.copy()
        prior[0,3] += phase+.045
        seed_poses.append(prior)
        case(f'seam_single_{phase}','narrow_repeat_single',narrow,narrow_scan,[prior],'reject', seam_bias_m=phase)
    case('seam_all','narrow_repeat_compared',narrow,narrow_scan,seed_poses,'reject')
    unique = passage(rng, 7800, 10, 3.4, 2.8, blocks=True)
    correct_map = target('reference_A',unique)
    pts = transform(passage(rng, 2800, 4.5, 3.4, 2.8, blocks=True),np.linalg.inv(T))
    scan = save('site_query',pts)
    perturbed = T.copy()
    perturbed[:3,3] += [.08,.02,.02]
    for fraction in [.1,.3,.5]:
        n = round(len(pts)*fraction/(1-fraction))
        noise = rng.uniform([-5,-2.7,-1.9], [5,2.7,2.8], (n,3))
        xyz = np.vstack([pts,noise])
        rng.shuffle(xyz)
        mixed = save(f'scatter_{fraction}',xyz)
        case(f'outliers_{fraction}', 'uniform_scattered_outliers',correct_map,mixed,[perturbed],'accept',
             injected_fraction=n/len(xyz), inlier_points=len(pts))
    # Predefined synthetic survey: physically distinct corridor B is displaced 32 m laterally.
    # Alias is the same place as A, with a map-local origin translated by [6,-2,1].
    B = unique.copy()
    B[:,1] += .017*np.sin(B[:,0])
    near_map = target('similar_B',B)
    site_A, site_B = np.eye(4), pose([0,32,0])
    site_alias = pose([6,-2,1])
    alias_map = target('known_alias_A',transform(unique,np.linalg.inv(site_alias)))
    for name,m,site,place,prior in [('A',correct_map,site_A,'survey_corridor_A',perturbed),
                                    ('B',near_map,site_B,'survey_corridor_B',perturbed),
                                    ('alias_A',alias_map,site_alias,'survey_corridor_A',np.linalg.inv(site_alias)@perturbed)]:
        case('site_'+name,'similar_maps',m,scan,[prior], 'reject' if name=='B' else 'accept',
             T_site_map=site.tolist(), place_id=place, map_identity=name, T_site_source_truth=T.tolist())
        if name=='alias_A':
            cases[-1]['truth'] = (np.linalg.inv(site_alias)@T).tolist()
        if name=='B':
            cases[-1]['truth'] = None
        cases[-1]['stamp_ns'] = 1800000000000000100
    write_json(out/'challenge.json',dict(seed=SEED,cases=cases))
    return cases


def multi_map(rows, cases, elapsed):
    """Compare estimates in one surveyed site frame; known physical aliases may agree."""
    reps = []
    for row, c in zip(rows,cases):
        r = dict(row['replies'][row['selected']])
        if r.get('T_target_source') is not None:
            r['T_target_source']=(np.array(c['T_site_map']) @ r['T_target_source']).tolist()
        reps.append(r)
    choice,ok,reason = select_candidate(reps,elapsed,PROTOCOL)
    return dict(accepted=ok,reason=reason,selected_map=cases[choice]['map_identity'],
                place_id=cases[choice]['place_id'], site_candidates=[r.get('T_target_source') for r in reps],
                candidate_maps=[c['map_identity'] for c in cases], elapsed_ms=elapsed)


def execute(binary, data, out, frozen):
    if frozen['binary_sha256'] != digest(binary) or frozen['protocol'] != PROTOCOL:
        raise ValueError('challenge requires the frozen binary and thresholds')
    write_json(out/'supplemental_freeze.json',dict(base=frozen,seed=SEED,challenge_sha256=digest(__file__),
                                                  manifest_sha256=digest(data/'challenge.json')))
    cases=read_json(data/'challenge.json')['cases']
    with (out/'raw.jsonl').open('x',encoding='utf-8') as stream, (out/'cli.stderr').open('x') as log:
        for backend in ['icp','gicp']:
            engine=Engine(binary,data,log,1)
            try:
                previous=None
                site={}
                for c in cases:
                    if c['map'] != previous:
                        loaded=engine.load(c)
                        previous=c['map']
                    row=engine.match(c,backend,1,PROTOCOL)
                    row['load']=loaded
                    if c['scene']=='similar_maps':
                        site[c['map_identity']]=(row,c)
                        estimate=row['replies'][0].get('T_target_source')
                        row['site_error']=error(np.array(c['T_site_map'])@estimate,c['T_site_source_truth']) if estimate is not None else None
                    write_row(stream,dict(row,scene=c['scene'],expected=c['expected'],load=loaded,kind='single'))
                for ids in [['A','B'], ['A','alias_A']]:
                    paired=[site[k] for k in ids]
                    result=multi_map([p[0] for p in paired],[p[1] for p in paired],sum(p[0]['wall_ms'] for p in paired))
                    write_row(stream,dict(result,backend=backend,kind='cross_map',case='+'.join(ids),
                                          total_with_cold_loads_ms=result['elapsed_ms']+sum(p[0]['load']['wall_ms'] for p in paired)))
            finally:
                engine.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--cli',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--freeze',type=Path,required=True)
    a=parser.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    data=(a.out/'data').resolve()
    generate(data)
    execute(a.cli.resolve(),data,a.out,read_json(a.freeze))
