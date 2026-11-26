"""Exercise the installed JSONL transport, both backends, and map generation isolation."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def pose(x=0.0, y=0.0, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return [[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]]


def pcd(path, points):
    with path.open("w", encoding="ascii", newline="\n") as f:
        n = len(points)
        f.write(f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\nPOINTS {n}\nDATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def near(a, b, epsilon):
    return max(abs(v - w) for ra, rb in zip(a, b) for v, w in zip(ra, rb)) < epsilon


def main(executable, directory):
    proc = subprocess.Popen([executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            text=True, encoding="utf-8")
    def ask(request):
        line = request if isinstance(request, str) else json.dumps(request, allow_nan=False)
        proc.stdin.write(line + "\n")
        proc.stdin.flush()
        response = json.loads(proc.stdout.readline())
        assert isinstance(response["ok"], bool) and isinstance(response["accepted"], bool), response
        return response

    grid = [i * 0.18 for i in range(-10, 11)]
    plane = [(x, y, 0) for x in grid for y in grid]
    pts = plane + [(1.9, x, y) for x in grid for y in grid] + [(x, -2.1, y) for x in grid for y in grid]
    path = directory / "map.pcd"
    metadata = dict(op="load", id="room", version="test", sha256=pcd(path, pts),
                    path=str(path), frame="map", units="m", gravity="+z", origin="test fixture")
    try:
        assert not ask("{")["ok"]
        load = ask(metadata)
        assert load["ok"] and load["generation"] == 1, load
        failed = ask(dict(metadata, sha256="0"*64))
        assert not failed["ok"] and failed["generation"] == 1, failed
        req = dict(op="match", scan_path=str(path), scan_id="f1", stamp_ns=10**18+11,
                   source_frame="base_link", backend="icp", threads=1, radius=8.,
                   budget_ms=10000., initial=pose(), local=pose(.2, -.3, .23), generation=1)
        inverse = pose(-.2*math.cos(.23)+.3*math.sin(.23), .2*math.sin(.23)+.3*math.cos(.23), -.23)
        for backend in ["icp", "gicp"]:
            res = ask(dict(req, backend=backend))
            assert res["accepted"], (backend, res)
            assert res["stamp_ns"] == req["stamp_ns"] and res["quality"]["source_finite_denominator"] == len(pts)
            assert near(res["T_map_W"], inverse, 0.025) and near(res["T_target_source"], pose(), .025), res
            assert res["peak_rss_kib_process"] > 0 and res["inlier_ratio"] > .99 and res["weak_ratio"] > .004
        assert ask(dict(req, backend="gicp"))["stages"]["gicp"]["target_cache_hit"]
        shifted = ask(dict(req, initial=pose(.2)))["submap"]
        assert shifted["cached"] and shifted["center_m"] != shifted["cache_center_m"]
        assert shifted["effective_radius_m"] >= shifted["radius_m"] + math.sqrt(3)*shifted["grid_m"]/2
        across = ask(dict(req, initial=pose(.6)))["submap"]
        assert not across["cached"] and across["entries"] == 2 and across["cache_center_m"] != shifted["cache_center_m"]
        assert ask(dict(req, radius=7.8))["submap"]["evictions"] == 1
        assert ask(dict(req, radius=7.8))["submap"]["cached"]
        assert ask(dict(req, generation=2))["reason"] == "stale_generation"
        assert ask(dict(req, now_ns=req["stamp_ns"]+500000000, max_age_ms=20))["reason"] == "stale_stamp"
        assert ask(dict(req, now_ns=req["stamp_ns"]-1, max_age_ms=20))["reason"] == "stale_stamp"
        assert ask(dict(req, budget_ms=.0001))["reason"] == "time_budget"
        assert not ask(dict(req, initial=[[1]]))["ok"]
        mirror = pose()
        mirror[2][2] = -1
        assert not ask(dict(req, initial=mirror))["ok"]
        assert not ask(dict(req, points=[]))["ok"]
        inline = {k: v for k,v in req.items() if k != "scan_path"}
        assert ask(dict(inline, points=[]))["reason"] == "insufficient_scan"
        assert ask(dict(inline, points=[[0,0,0]]*10))["reason"] == "insufficient_scan"
        assert ask(dict(req, initial=pose(80)))["reason"] == "insufficient_submap"
        distant = [(.1+i*.01, 5, 7) for i in range(20)]
        partial = ask(dict(inline, points=pts+distant))
        quality = partial["quality"]
        assert quality["distance_outliers"] >= len(distant) and quality["inlier_ratio"] < 1.0
        assert quality["source_finite_denominator"] == quality["inliers"]+quality["distance_outliers"]+quality["no_nearest"]
        assert quality["all_nearest_max_m"] > quality["inlier_max_m"]
        flat = directory / "plane.pcd"
        flatmeta = dict(metadata, id="floor", path=str(flat), sha256=pcd(flat, plane))
        assert ask(flatmeta)["generation"] == 2
        assert ask(req)["reason"] == "stale_generation"
        for backend in ["icp", "gicp"]:
            weak = ask(dict(req, generation=2, backend=backend, scan_path=str(flat)))
            assert not weak["accepted"] and "weak_geometry" in weak["failed_gates"], weak
            assert weak["submap"]["generation"]==2 and weak["submap"]["evictions"]==0
        # Two isolated perpendicular vertical patches determine x/y/yaw; translation along z is unobservable.
        walls = [(3, i, j) for i in grid for j in grid] + [(i, 3, j) for i in grid for j in grid]
        vertical = directory / "vertical.pcd"
        assert ask(dict(metadata, id="vertical", path=str(vertical), sha256=pcd(vertical, walls)))["generation"] == 3
        for backend in ["icp", "gicp"]:
            weak = ask(dict(req, generation=3, backend=backend, scan_path=str(vertical)))
            assert weak["weak_ratio"] >= weak["limits"]["weak_ratio_min"], weak
            assert not weak["accepted"] and "weak_se3" in weak["failed_gates"], weak
            assert abs(weak["quality"]["weak_direction_se3"][2]) > .99, weak
        print("JSONL contract, transforms, ICP/GICP, residual denominator, degeneracy, cache, hash and age checks passed")
    finally:
        proc.stdin.close()
        proc.wait(timeout=10)
        assert proc.returncode == 0


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as tmp:
        main(os.path.abspath(sys.argv[1]), Path(tmp))
