"""Independent longitudinal-axis sweep, without using a registration estimator."""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from common import cloud, digest, read_json, transform, write_json


def sweep(data):
    dataset = read_json(data / 'dataset.json')
    audit = []
    for c in dataset['cases']:
        if c['scene'] != 'parallel_walls' or c['longitudinal_prior_shift'] != 0:
            continue
        src = transform(cloud(data / c['scan']), c['truth'])
        target = cloud(data / c['map'])
        # Infinite analytic planes are known from the generator. Use the visible plane normal for each point.
        ymin, zmin = target[:, 1:].min(axis=0)
        ymax, zmax = target[:, 1:].max(axis=0)
        def plane_distances(points):
            return np.stack([abs(points[:,1]-ymin), abs(points[:,1]-ymax),
                             abs(points[:,2]-zmin), abs(points[:,2]-zmax)], axis=1)
        face = plane_distances(src).argmin(axis=1)
        normals = np.array([[0,-1,0],[0,1,0],[0,0,-1],[0,0,1]])[face]
        lever = src - np.array(c['truth'])[:3,3]
        scale = np.sqrt(np.mean(np.sum(lever[:,:2]**2, axis=1)))
        J = np.c_[normals[:,:2], np.cross(lever, normals)[:,2]/scale]
        H = J.T @ J / len(J)
        tree = cKDTree(target)
        for dx in [-4, -2, -1, -.5, 0, .5, 1, 2, 4]:
            shifted = src + [dx,0,0]
            d = tree.query(shifted)[0]
            prior = np.asarray(c['truth'])[:3,3] + [dx,0,0]
            audit.append(dict(split=c['split'], shift_m=dx,
                              nearest_rmse_m=np.sqrt(np.mean(d*d)), nearest_p95_m=np.percentile(d,95),
                              analytic_plane_rmse_m=np.sqrt(np.mean(np.min(plane_distances(shifted),axis=1)**2)),
                              end_clearance_m=min(shifted[:,0].min()-target[:,0].min(),target[:,0].max()-shifted[:,0].max()),
                              crop_clearance_m=12.-np.max(np.linalg.norm(shifted-prior,axis=1)),
                              normal_eigenvalues=np.linalg.eigvalsh(H).tolist(),
                              axial_information=float(H[0,0]), axis='map +X'))
    assert all(r['end_clearance_m'] > 12 and r['crop_clearance_m'] > 3 and
               r['analytic_plane_rmse_m'] < 1e-5 and r['axial_information'] == 0 for r in audit)
    return {'dataset_sha256': digest(data / 'dataset.json'), 'sweep': audit}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('data', type=Path)
    parser.add_argument('output', type=Path)
    a = parser.parse_args()
    write_json(a.output, sweep(a.data))
