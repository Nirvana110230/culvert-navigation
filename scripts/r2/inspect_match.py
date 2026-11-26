"""从归档的逐次 NPZ 和实际地图导出对齐前、对齐后及原始目标裁区，单位 m。"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
here=Path(__file__).resolve()
project=next(parent for parent in here.parents if (parent/'benchmarks/r2/common.py').is_file())
sys.path.insert(0,str(project/'benchmarks/r2'))
from common import cloud,digest,save_cloud,transform
p=argparse.ArgumentParser()
p.add_argument('run',type=Path)
p.add_argument('fixture',type=Path)
p.add_argument('out',type=Path)
p.add_argument('--stamp',type=int)
a=p.parse_args()
records=[json.loads(s) for s in (a.run/'frames.jsonl').read_text(encoding='utf8').splitlines()]
row=next(r for r in records if r['kind']=='match' and (r['stamp_ns']==a.stamp if a.stamp else r['accepted']))
raw=row['result']
assert raw.get('T_target_source') is not None
meta=json.loads((a.run/'map.json').read_text(encoding='utf8'))
target=a.fixture/Path(meta['path']).name
assert digest(target)==row['map_sha256']==meta['sha256']
data=np.load(a.run/row['input_path'],allow_pickle=False)
assert int(data['stamp_ns'])==row['stamp_ns']
a.out.mkdir(parents=True,exist_ok=False)
source=data['points']
points=cloud(target)
sub=raw['submap']
center=np.asarray(sub['cache_center_m'])
radius=sub['effective_radius_m']
save_cloud(a.out/'target_raw_crop.pcd',points[np.sum((points-center)**2,axis=1)<=radius**2])
save_cloud(a.out/'source_prior.pcd',transform(source,data['initial']))
save_cloud(a.out/'source_candidate.pcd',transform(source,np.asarray(raw['T_target_source'])))
(a.out/'record.json').write_text(json.dumps(dict(run=a.run.name,map=meta,frame=row,
    target_export='raw map points in cache sphere, before backend voxel/covariance filter'),indent=2),encoding='utf8')
