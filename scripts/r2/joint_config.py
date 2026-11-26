"""从冻结扫描清单构造同袋软件回归先验，隔离 run 内保存完整配置。"""
import os
import json
import sys
from pathlib import Path
import yaml
import numpy as np
fixtures,scene,backend,run,mode = sys.argv[1:]
fixtures,run = Path(fixtures),Path(run)
source = json.loads((fixtures/'history.json').read_text())['runs']['static' if scene=='bad' else scene]
case = source['replay']
manifest = dict(path=str(fixtures/case['map']),id=case['map_frame'],version=source['run'],sha256=case['map_sha256'],
    frame=case['map_frame'],units='m',gravity=source['gravity'],origin=source['origin'])
(run/'map.json').write_text(json.dumps(manifest,indent=2))
seed=case['initial']
if mode=='B':
    adapter=Path(os.environ['NAV_PREFIX']) / 'share/culvert_livo_adapter/config/isolated_test.yaml'
    cfg=yaml.safe_load(adapter.read_text())['culvert_livo_adapter']['ros__parameters']
    assert cfg['calibration_id']=='SYNTHETIC_TEST_ONLY' and cfg['imu_from_base_xyzw']==[0,0,0,1]
    imu_from_base=np.eye(4)
    imu_from_base[:3,3]=cfg['imu_from_base_xyz_m']
    seed=(np.array(seed)@np.linalg.inv(source['T_I_L'])@imu_from_base).tolist()
params = dict(mode=mode,map_manifest=str(run/'map.json'),initial_pose_json=json.dumps(seed),
    imu_from_lidar_json=json.dumps(source['T_I_L']),backend=backend,match_hz=2.0,threads=2,budget_ms=500.,
    record_path=str(run/'frames.jsonl'),save_inputs=True,use_sim_time=True)
if mode=='B':
    params['imu_from_lidar_json']=''
(run/'localizer.yaml').write_text(yaml.safe_dump({'culvert_localization':{'ros__parameters':params}}))
