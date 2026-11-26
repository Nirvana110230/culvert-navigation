"""New tests for the checked C++ client and the frozen worker's failed-load transaction."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
from protocol import pcd, pose


def exercise(mode, executable, root):
    grid = [i * .18 for i in range(-10, 11)]
    xyz = [(a, b, 0) for a in grid for b in grid]
    xyz += [(1.9, a, b) for a in grid for b in grid]
    xyz += [(a, -2.1, b) for a in grid for b in grid]
    path = root / "good.pcd"
    digest = pcd(path, xyz)
    meta = dict(op="load", path=str(path), id="laboratory", version="v2", sha256=digest,
                units="m", frame="map_lab", gravity="+Z up", origin="laboratory")
    req = dict(op="match", scan_path=str(path), scan_id="query-a", stamp_ns=10**18+5,
               source_frame="sensor", initial=pose(), local=pose(.2,-.3,.23),
               radius=8, threads=1, budget_ms=5000, backend="icp", generation=1)
    with subprocess.Popen([executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                          text=True, encoding="utf8") as proc:
        def ask(value):
            proc.stdin.write(json.dumps(value, allow_nan=False)+"\n")
            proc.stdin.flush()
            return json.loads(proc.stdout.readline())

        try:
            loaded = ask(meta)
            assert loaded["ok"] and loaded["generation"]==1, loaded
            if mode=="invalid-maps":
                # Recompute each bad file's real SHA, so rejection exercises PCD handling.
                good = path.read_bytes()
                header, rows = good.split(b"DATA ascii\n",1)
                first, remaining = rows.split(b"\n",1)
                integer_rows = b"".join(b" ".join(row.split()[:2])+b" 0\n" for row in rows.splitlines())
                replacements = {
                    "empty": (b"", "pcd_invalid_dimensions"),
                    "zero-points": (b"VERSION .7\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n", "pcd_invalid_dimensions"),
                    "damaged-header": (good.replace(b"HEIGHT 1", b"HEIGHT 2", 1), "pcd_invalid_dimensions"),
                    "missing-z": (good.replace(b"FIELDS x y z", b"FIELDS x y q", 1), "pcd_xyz_float32_required"),
                    "unsupported-type": (header.replace(b"TYPE F F F",b"TYPE F F I",1)+b"DATA ascii\n"+integer_rows, "pcd_xyz_float32_required"),
                    "nonfinite": (header+b"DATA ascii\nnan "+b" ".join(first.split()[1:])+b"\n"+remaining, "nonfinite_points"),
                }
                for key,(data,reason) in replacements.items():
                    bad=root/(key+".pcd")
                    bad.write_bytes(data)
                    out=ask(dict(meta, id=key, path=str(bad), sha256=hashlib.sha256(data).hexdigest()))
                    assert not out["ok"] and out["reason"]==reason and out["generation"]==1,(key,out)
                    retained=ask(req)
                    assert retained["accepted"] and retained["map_id"]==meta["id"] and retained["map_sha256"]==digest,retained
                print("Empty, zero-point, malformed dimensions, missing z, integer z, nonfinite: original map verified after each failure.")
                return

            assert mode=="binding"
            req.update(map_id=meta["id"], version=meta["version"], sha256=digest, target_frame=meta["frame"])
            for name,bad,reason in (("map_id","other","map_id_mismatch"),
                                     ("version","v1","map_version_mismatch"),
                                     ("sha256","0"*64,"map_sha256_mismatch"),
                                     ("generation",2,"stale_generation"),
                                     ("target_frame","other_axes","target_frame_mismatch")):
                for revised,expected in ((dict(req,**{name:bad}),reason),
                                          ({k:v for k,v in req.items() if k!=name},
                                           "missing_or_invalid_"+name)):
                    r=ask(revised)
                    assert not r["ok"] and not r["accepted"] and r["reason"]==expected,r
                    assert not r["computation_completed"] and not r["valid_se3"] and r["candidate"] is None,r
                    assert r["geometry_gate_passed"] is None and r["prior_gate_passed"] is None,r
                    assert r["T_target_source"] is None and r["T_map_W"] is None and r["submap_id"] is None,r
            submap=None
            invalid_pose=ask(dict(req, initial=[[1]]))
            assert invalid_pose["backend_version"].startswith("pcl-") and invalid_pose["candidate"] is None,invalid_pose
            for backend,version in (("icp","pcl-"),("gicp","small_gicp@fa0cfc98")):
                r=ask(dict(req,backend=backend))
                assert r["accepted"] and r["computation_completed"] and r["valid_se3"] and r["backend_version"].startswith(version),r
                assert r["geometry_gate_passed"] and r["prior_gate_passed"] and r["candidate"]["accepted"],r
                assert r["candidate"]["T_map_W"]==r["T_map_W"] and r["stamp_ns"]==req["stamp_ns"]
                assert not r["absolute_truth_available"] and r["absolute_pose_error"] is None,r
                elements=json.loads(r["submap_id"])
                assert elements[:5]==[meta["id"], meta["version"], digest, meta["frame"],1],elements
                assert elements[5:]==[r["submap"][k] for k in ("cache_center_m","grid_m","radius_m","effective_radius_m")]
                if submap is not None:
                    assert r["submap_id"]==submap
                submap=r["submap_id"]
            inline={k:v for k,v in req.items() if k!="scan_path"}
            # 25 identical points pass raw minimum but collapse to one voxel. The worker
            # may carry its initial transformation for diagnostics; never promote it.
            collapsed=ask(dict(inline,points=[[1,1,1]]*25))
            assert not collapsed["accepted"] and collapsed["candidate"] is None and not collapsed["valid_se3"],collapsed
            assert not collapsed["computation_completed"] and collapsed["T_target_source"] is None
            bad_load=ask(dict(meta,sha256="0"*64))
            assert not bad_load["ok"] and bad_load["generation"]==1
            assert ask(req)["accepted"]
            assert ask(meta)["generation"]==2
            assert ask(req)["reason"]=="stale_generation"
            switched=ask(dict(req,generation=2))
            assert switched["accepted"] and switched["submap_id"]!=submap
            print("Bindings, source-to-map candidates, both backends, unevaluated gates, unknown truth and map generations verified.")
        finally:
            proc.stdin.close()
            proc.wait(timeout=10)
            assert proc.returncode==0


if __name__=="__main__":
    with tempfile.TemporaryDirectory() as directory:
        exercise(sys.argv[1],sys.argv[2],Path(directory))
