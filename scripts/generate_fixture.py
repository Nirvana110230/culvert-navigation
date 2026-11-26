"""Generate a non-symmetric corridor and an exact endpoint lidar scan for backend tests."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def save_cloud(path,xyz):
    header=['VERSION .7','FIELDS x y z','SIZE 4 4 4','TYPE F F F','COUNT 1 1 1',
            f'WIDTH {len(xyz)}','HEIGHT 1','VIEWPOINT 0 0 0 1 0 0 0',f'POINTS {len(xyz)}','DATA binary','']
    path.write_bytes('\n'.join(header).encode('ascii')+xyz.astype('<f4').tobytes())


def main(folder):
    folder.mkdir(parents=True,exist_ok=True)
    x=np.linspace(0,12,61);z=np.linspace(0,2.5,14);y=np.linspace(-1.5,1.5,17)
    xx,zz=np.meshgrid(x,z); fx,fy=np.meshgrid(x,y)
    stack=lambda a,b,c:np.column_stack((a.ravel(),b.ravel(),c.ravel()))
    walls=[stack(xx,np.full_like(xx,v),zz) for v in (-1.5,1.5)]
    floor=stack(fx,fy,np.zeros_like(fx))
    # A non-coplanar ledge breaks the straight corridor's translational ambiguity.
    u,v=np.meshgrid(np.linspace(0,1,10),np.linspace(0,1,9))
    feature=stack(6.8+.32*u+.15*v,-1.45+.67*v,.65+u)
    xyz=np.unique(np.vstack([*walls,floor,feature]),axis=0)
    a,b,c=np.deg2rad([13,-7,28])
    ca,sa=np.cos(a),np.sin(a);cb,sb=np.cos(b),np.sin(b);cc,sc=np.cos(c),np.sin(c)
    R=np.array([[cc,-sc,0],[sc,cc,0],[0,0,1]])@np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]])@np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]])
    position=np.array([4.2,-.3,.8])
    mask=(xyz[:,0]>=3)&(xyz[:,0]<=9)
    scan=(xyz[mask]-position)@R
    save_cloud(folder/'map.pcd',xyz);save_cloud(folder/'scan.pcd',scan)
    # Independent round trip on the serialized float32 coordinates.
    read=lambda p:np.frombuffer(p.read_bytes().split(b'DATA binary\n',1)[1],dtype='<f4').reshape(-1,3)
    err=float(np.max(np.abs(read(folder/'scan.pcd')@R.T+position-read(folder/'map.pcd')[mask])))
    assert err<1e-6
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=position
    truth=dict(dataset='synthetic_round1_v1',frames=['map_d','lidar_d'],T_map_d_lidar_d=T.tolist(),units='m',
               map_points=len(xyz),scan_points=len(scan),map_row_indices=np.flatnonzero(mask).tolist(),
               serialized_max_abs_error_m=err,sha256={n:hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in ('map.pcd','scan.pcd')})
    (folder/'truth.json').write_text(json.dumps(truth,indent=2)+'\n')
    print(truth['map_points'],truth['scan_points'],err)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('output',type=Path)
    main(parser.parse_args().output)
