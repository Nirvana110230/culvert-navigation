"""Summarize one isolated replay using Python's standard library. No ROS installation required."""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct


STATE = "/culvert_source/state"
SCAN = "/culvert_source/scan"
ODOM = "/culvert_test/odom"
CLOUD = "/culvert_test/localization_cloud"
IMU = "/culvert_test/imu"
RAW_IMU = "/livox/imu"
TF = "test_odom -> test_base_link"
NATIVE = "/aft_mapped_to_init"
NS = 10**9
COMPARISON_LIMITS = {"all_xyz_m": 1e-5, "base_position_m": 1e-8,
                     "base_rotation_matrix": 1e-8, "tf_position_m": 1e-8,
                     "tf_rotation_matrix": 1e-8}


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write_json(path, value):
    temp = path.with_suffix(".json.tmp")
    with temp.open("w", encoding="utf-8", newline="\n") as out:
        json.dump(value, out, indent=2, ensure_ascii=False, allow_nan=False)
        out.write("\n")
    temp.replace(path)


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as src:
        return list(csv.DictReader(src))


def number(value):
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def distribution(values):
    vals = sorted(x for x in values if x is not None and math.isfinite(x))
    if not vals:
        return {"n": 0}
    def quantile(q):
        i = (len(vals) - 1) * q
        left = int(i)
        return vals[left] + (vals[min(left + 1, len(vals) - 1)] - vals[left]) * (i - left)
    return {"n": len(vals), "min": vals[0], "p50": quantile(.5), "p95": quantile(.95),
            "p99": quantile(.99), "max": vals[-1], "mean": statistics.fmean(vals)}


def ratio(n, d):
    return {"numerator": n, "denominator": d,
            "fraction": n / d if n is not None and d is not None and d > 0 else None}


def differences(xs):
    return (b - a for a, b in zip(xs, xs[1:]))


def stamp_domain(name):
    if name in (STATE, SCAN, ODOM, CLOUD, TF):
        return "估计器切片终点"
    if name in (RAW_IMU, IMU):
        return "Livox IMU 量测域"
    if name in ("/livox/lidar", "/hik_camera/image_timeshare"):
        return "传感器消息 header"
    if " -> " in name and "imu" in name and "base_link ->" in name:
        return "静态 TF 建立时刻"
    if name in ("/diagnostics", "/culvert_test/diagnostics"):
        return "诊断节点发布时 ROS 时钟，暂停时允许同戳"
    if name == "/LIVO2/imu_propagate":
        return "IMU 传播时间"
    if name in (NATIVE, "/cloud_registered", "/path", "/cloud_effected", "camera_init -> aft_mapped"):
        return "本试验三传感器 LIVO 原生发布时 ROS 时钟"
    return "消息 header，未独立判定物理域"


def rate(series, scale):
    if len(series) < 2:
        return {"n": len(series), "hz": None}
    deltas = list(differences(series))
    duration = (series[-1] - series[0]) * scale
    return {"n": len(series), "first": series[0], "last": series[-1], "span_s": duration,
            "hz": (len(series) - 1) / duration if duration > 0 and min(deltas) >= 0 else None,
            "duplicate_adjacent": deltas.count(0), "backward_adjacent": sum(d < 0 for d in deltas),
            "positive_interval_ms": distribution(d * scale * 1000 for d in deltas if d > 0)}


def exact_pair(left, right):
    left_times, right_times = defaultdict(list), defaultdict(list)
    for ns, arrival in left:
        left_times[ns].append(arrival)
    for ns, arrival in right:
        right_times[ns].append(arrival)
    common = sorted(left_times.keys() & right_times.keys())
    waits = [1000 * (b - a) for ns in common for a, b in zip(left_times[ns], right_times[ns])]
    ordered_right = sorted(right_times)
    nearest = []
    if ordered_right:
        for ns, _ in left:
            i = bisect_left(ordered_right, ns)
            candidates = ordered_right[max(0, i-1):i+1]
            nearest.append(min(abs(v - ns) for v in candidates))
    return {"left_n": len(left), "right_n": len(right), "unique_common_stamps": len(common),
            "exact_n": len(waits), "exact_stamp_diff_ns": 0 if waits else None,
            "left_unmatched": len(left)-len(waits), "right_unmatched": len(right)-len(waits),
            "right_arrival_minus_left_ms_on_exact_pair": distribution(waits),
            "left_to_nearest_right_abs_ns": distribution(nearest)}


def unpack_pose(row):
    stamp = int(row[0])
    if isinstance(row[1], (tuple, list)):
        pos, q = row[1:3]
        twist = row[3] if len(row) > 3 else []
    else:
        pos, q, twist = row[1:4], row[4:8], row[8:14]
    if len(pos) != 3 or len(q) != 4 or len(twist) not in (0, 6):
        raise ValueError("不支持的位姿记录形状")
    return {"stamp_ns": stamp, "xyz_m": pos, "xyzw": q, "twist": twist or None}


def normalized(q):
    norm = math.hypot(*q)
    if norm == 0 or not math.isfinite(norm):
        return None
    return [x / norm for x in q]


def angular_distance(a, b):
    a, b = normalized(a), normalized(b)
    if a is None or b is None:
        return None
    if sum(x*y for x, y in zip(a, b)) < 0:
        b = [-x for x in b]
    # Handles quaternion signs and preserves precision near zero rotation.
    return 4 * math.atan2(math.dist(a, b), math.hypot(*(x+y for x, y in zip(a, b))))


def valid_pose(p):
    return (all(math.isfinite(x) for x in p["xyz_m"]) and
            abs(math.hypot(*p["xyzw"]) - 1) <= .05 and p["stamp_ns"] > 0)


def trajectory(poses, twist_is_estimated):
    displacements, angles, speeds, angular_speeds, from_first = [], [], [], [], []
    largest_step, largest_angle = None, None
    qflip, invalid_time = 0, 0
    valid = [p for p in poses if valid_pose(p)]
    for a, b in zip(poses, poses[1:]):
        if not valid_pose(a) or not valid_pose(b):
            continue
        dt = (b["stamp_ns"] - a["stamp_ns"]) / NS
        if dt <= 0:
            invalid_time += 1
            continue
        step = math.dist(a["xyz_m"], b["xyz_m"])
        angle = angular_distance(a["xyzw"], b["xyzw"])
        displacements.append(step)
        angles.append(angle)
        speeds.append(step / dt)
        angular_speeds.append(angle / dt)
        qflip += int(sum(x*y for x,y in zip(a["xyzw"],b["xyzw"])) < 0)
        if largest_step is None or step > largest_step["value"]:
            largest_step = {"value": step, "from_ns": a["stamp_ns"], "to_ns": b["stamp_ns"], "dt_s": dt}
        if largest_angle is None or angle > largest_angle["value"]:
            largest_angle = {"value": angle, "from_ns": a["stamp_ns"], "to_ns": b["stamp_ns"], "dt_s": dt}
    if valid:
        from_first = [math.dist(valid[0]["xyz_m"], p["xyz_m"]) for p in valid]
    twists = [p["twist"] for p in valid if p["twist"] and all(math.isfinite(v) for v in p["twist"])]
    return {"pose_valid": ratio(len(valid), len(poses)), "adjacent_positive_dt": ratio(len(speeds), max(len(poses)-1,0)),
            "nonpositive_dt": invalid_time, "quaternion_abs_norm_error": distribution(
                abs(math.hypot(*p["xyzw"])-1) for p in poses),
            "quaternion_representation_sign_changes": qflip, "step_m": distribution(displacements),
            "step_rotation_rad": distribution(angles), "max_step_m": largest_step,
            "max_step_rad": largest_angle, "displacement_from_first_m": distribution(from_first),
            "end_to_start_m": from_first[-1] if from_first else None,
            "path_length_sum_valid_adjacent_m": sum(displacements) if displacements else None,
            "pose_difference_speed_mps": distribution(speeds),
            "pose_difference_angular_speed_radps": distribution(angular_speeds),
            "published_body_linear_speed_mps": distribution(math.hypot(*t[:3]) for t in twists)
                if twist_is_estimated else None,
            "published_body_angular_speed_radps": distribution(math.hypot(*t[3:]) for t in twists)
                if twist_is_estimated else None}


def resources(rows):
    result = {"series_samples": len(rows), "scope": "采样器自身壁钟，包括初始化、输入尾部及退出等待",
              "max_thermal_zone_c": distribution(number(r.get("max_thermal_zone_c")) for r in rows)}
    for prefix in ("mapper", "adapter"):
        series = [(number(r.get("seconds")), number(r.get(prefix+"_cpu_s")),
                   number(r.get(prefix+"_rss_kib"))) for r in rows]
        series = [(t,c,r/1024) for t,c,r in series if None not in (t,c,r)]
        intervals = [(b[0]-a[0], b[1]-a[1]) for a,b in zip(series,series[1:]) if b[0]>a[0] and b[1]>=a[1]]
        rss = [r for _,_,r in series]
        rss_slope = None
        if len(series) >= 2:
            t_mean, r_mean = statistics.fmean(t for t,_,_ in series), statistics.fmean(rss)
            den = sum((t-t_mean)**2 for t,_,_ in series)
            if den:
                rss_slope = sum((t-t_mean)*(r-r_mean) for t,_,r in series) / den
        result[prefix] = {"sample_n": len(series), "interval_cpu_pct_one_core": distribution(
            100*dc/dt for dt,dc in intervals), "mean_cpu_pct_one_core_weighted": (
            100*sum(dc for _,dc in intervals)/sum(dt for dt,_ in intervals) if intervals else None),
            "span_s": series[-1][0]-series[0][0] if series else None,
            "rss_mib": distribution(rss), "rss_first_mib": rss[0] if rss else None,
            "rss_last_mib": rss[-1] if rss else None, "rss_change_mib": rss[-1]-rss[0] if rss else None,
            "rss_peak_above_first_mib": max(rss)-rss[0] if rss else None, "rss_ols_mib_per_s": rss_slope}
    return result


def diagnostics(rows):
    result = {"samples": len(rows), "level_counts": dict(Counter(str(r.get("level")) for r in rows)),
              "message_counts": dict(Counter(str(r.get("message")) for r in rows))}
    if not rows:
        return result
    last = rows[-1]
    result["last_snapshot"] = last
    result["last_reasons_cumulative"] = {k[7:]: int(v) for k,v in last.items() if k.startswith("reject.")}
    result["last_totals"] = {k: int(last[k]) for k in ("rejects","duplicate_count","rollback_count",
          "other_reject_count","pair_expired","queue_overflow","matched_pairs") if k in last}
    result["last_window_percentiles_scope"] = "最近至多256次该类事件的节点分位数，均值与最大值自进程启动累计；最后诊断与停机间可有尾部事件"
    result["timing_ms"] = {}
    for key in ("callback_wall","state_callback_wall","cloud_callback_wall","imu_callback_wall",
                "cloud_conversion_wall","pair_wait"):
        metrics = {label: number(last.get(key+"_"+field+"_ms")) for label,field in (
            ("last_window_p50","p50"),("last_window_p95","p95"),("cumulative_mean","mean"),("cumulative_max","max"))}
        result["timing_ms"][key] = {k:(v if v is not None and v >= 0 else None) for k,v in metrics.items()}
    return result


def processing(rows):
    result = {"sampled_rows": len(rows), "scope": "约每壁钟1秒记录被抽中的一次同步循环，采样行不等于全部LIO/VIO更新"}
    for key in ("processing_ms","cycle_ms","lidar_queue","imu_queue","image_queue"):
        result[key] = distribution(number(r.get(key)) for r in rows)
    if rows:
        result["last_sampled_counters"] = {k:int(rows[-1][k]) for k in
            ("frames","synced","vio_frames","visual_update_frames","visual_update_steps")
                if rows[-1].get(k) not in (None,"")}
    return result


def unique_pose_map(rows):
    counts = Counter(p["stamp_ns"] for p in rows)
    return {p["stamp_ns"]: p for p in rows if counts[p["stamp_ns"]] == 1 and valid_pose(p)}


def rotate(q, v):
    x,y,z,w = normalized(q)
    def cross(a,b):
        return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    uv = cross((x,y,z), v)
    uuv = cross((x,y,z), uv)
    return [a + 2*(w*b+c) for a,b,c in zip(v,uv,uuv)]


def compare_poses(left, right, arm):
    positional, angular = [], []
    for stamp in left.keys() & right.keys():
        a,b = left[stamp],right[stamp]
        offset = rotate(a["xyzw"], arm)
        expected = [v+dv for v,dv in zip(a["xyz_m"], offset)]
        positional.append(math.dist(expected,b["xyz_m"]))
        angular.append(angular_distance(a["xyzw"],b["xyzw"]))
    return {"unique_valid_pairs":len(positional), "translation_l2_m":distribution(positional),
            "angle_rad":distribution(angular)}


def inventory_pcd(path):
    with path.open("rb") as f:
        header = {}
        for _ in range(64):
            line = f.readline(4096)
            if not line:
                raise ValueError("PCD header truncated: " + path.name)
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                break
        if (header.get("DATA") != ["binary"] or header.get("FIELDS") != ["x","y","z","intensity"]
                or header.get("SIZE") != ["4"]*4 or header.get("TYPE") != ["F"]*4
                or header.get("COUNT") != ["1"]*4):
            raise ValueError("样本非观察器的二进制 XYZI 布局: " + path.name)
        n = int(header["POINTS"][0])
        payload = f.read()
        if n*16 != len(payload) or n != int(header["WIDTH"][0])*int(header["HEIGHT"][0]):
            raise ValueError("样本点数与载荷不一致: " + path.name)
        finite = sum(all(math.isfinite(x) for x in point) for point in struct.iter_unpack("<4f", payload))
    return {"pcd":path.name, "sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
            "points":n, "finite_xyzi_points":finite}


def sample_index(run, src, out, tf, by_kind, stamps):
    directory = run / "samples"
    if not directory.is_dir():
        return []
    scans, clouds = (Counter(t for t,_ in stamps.get(k,[])) for k in (SCAN,CLOUD))
    records = []
    for path in sorted(directory.glob("*.pcd")):
        stamp = int(path.stem)
        item = inventory_pcd(path)
        item.update({"stamp_ns":stamp, "frame":"livo_lidar", "source_state":src.get(stamp),
                     "adapter_odom":out.get(stamp),"adapter_tf":tf.get(stamp),
                     "source_scan_seen_at_stamp":scans[stamp],"adapter_cloud_seen_at_stamp":clouds[stamp],
                     "observer_max_abs_component_error":{k:by_kind.get(k,{}).get(stamp)
                                                         for k in COMPARISON_LIMITS}})
        item["all_point_comparison_available"] = item["observer_max_abs_component_error"].get("all_xyz_m") is not None
        records.append(item)
    if records:
        write_json(directory / "index.json", {"samples":records, "scope":
            "PCD保存源LiDAR去畸变XYZI，姿态/TF仅按唯一且有效的精确源戳匹配；云误差来自观察器同刻全部点的最大分量绝对误差。未记录逐戳误差时为null。"})
    return records


def summarize(run, scene):
    obs, stamps, poses, diag, replay = [read_json(run/(n+".json")) for n in
                                         ("observation","stamps","poses","diagnostics","replay")]
    proc = read_csv(run/"Log"/"processing.csv")
    res = read_csv(run/"resources.csv")
    source_path = run/"Log"/"culvert_source_counts.json"
    source = read_json(source_path) if source_path.is_file() else None
    mapper_path = run/"Log"/"input_counters.json"
    mapper_counts = read_json(mapper_path) if mapper_path.is_file() else None
    pose_data = {k:[unpack_pose(row) for row in v] for k,v in poses.items()}
    src, out, tf = (unique_pose_map(pose_data.get(k,[])) for k in (STATE,ODOM,TF))
    observed = obs["counts"]
    topics = sorted(set(stamps) | set(observed) | set(replay.get("stored_counts",{})) |
                    {STATE,SCAN,ODOM,CLOUD,IMU,RAW_IMU,NATIVE,"/tf","/tf_static"})
    streams = {}
    for topic in topics:
        times = stamps.get(topic,[])
        streams[topic] = {"observer_count":observed.get(topic,len(times)),
                          "bag_stored_count":replay.get("stored_counts",{}).get(topic),
                          "player_sent_count":replay.get("published",{}).get(topic),
                          "frames":obs.get("frames",{}).get(topic),
                          "header_domain":stamp_domain(topic),
                          "header_ns":rate([r[0] for r in times],1/NS),
                          "observer_arrival_s":rate([r[1] for r in times],1)}
        if not times and topic in replay.get("stored_counts",{}) and topic not in observed:
            streams[topic]["observer_count"] = None
        if topic in ("/tf","/tf_static"):
            streams[topic]["edge_specific_rates"] = obs.get("tf",{}).get(topic,[])
    comparisons = {}
    pairs = [(STATE,SCAN),(ODOM,CLOUD),(STATE,ODOM),(SCAN,CLOUD),(ODOM,TF),(RAW_IMU,IMU)]
    for a,b in pairs:
        comparisons[a+" | "+b] = exact_pair(stamps.get(a,[]),stamps.get(b,[]))
    all_stamps = [Counter(t for t,_ in stamps.get(k,[])) for k in (STATE,SCAN,ODOM,CLOUD,TF)]
    exact_chain = all_stamps[0].copy()
    for group in all_stamps[1:]:
        exact_chain &= group
    comparisons["source_state_scan_adapter_odom_cloud_tf_chain"] = {
        "exact_observations":sum(exact_chain.values()), "unique_stamps":len(exact_chain),
        "over_observed_clouds":ratio(sum(exact_chain.values()),len(stamps.get(CLOUD,[])))}
    pair_error = {kind:{int(t):v for t,v in rows} for kind,rows in obs.get("paired_errors",{}).items()}
    samples = sample_index(run,src,out,tf,pair_error,stamps)
    candidate = bool(source or STATE in observed)
    all_lio = source["attempts_after_odom"]+source["empty_before_odom"] if source else None
    after = source["attempts_after_odom"] if source else None
    accepted = source["published_pairs"] if source else None
    cloud_out, odom_out = (observed.get(k,0) if candidate else None for k in (CLOUD,ODOM))
    info = {"all_lio_cycles":all_lio,"after_odom_attempts":after, "source_published_pairs":accepted,
            "mapper_counters":mapper_counts, "source_counters":source, "cloud_over_all_lio":ratio(cloud_out,all_lio),
            "odom_over_all_lio":ratio(odom_out,all_lio),"source_gate_over_all_lio":ratio(accepted,all_lio),
            "source_gate_over_after_odom":ratio(accepted,after),
            "cloud_over_published_source":ratio(cloud_out,accepted),
            "odom_over_published_source":ratio(odom_out,accepted),
            "cloud_over_observed_source":ratio(cloud_out,observed.get(SCAN)),
            "odom_over_observed_source":ratio(odom_out,observed.get(STATE)),
            "imu_over_observed_raw":ratio(observed.get(IMU),observed.get(RAW_IMU))}
    if source:
        info["source_rejected_over_all_lio"] = ratio(all_lio-accepted,all_lio)
        info["source_rejected_after_odom"] = ratio(after-accepted,after)
        info["reconciled_first_rejection_count"] = sum(source.get("first_rejection",{}).values())
    if diag:
        for key in ("state","cloud","imu"):
            last = diag[-1]
            if key+"_seen" in last and key+"_published" in last:
                info[key+"_adapter_published_over_received"] = ratio(int(last[key+"_published"]),int(last[key+"_seen"]))
    checks = {}
    for kind,tol in COMPARISON_LIMITS.items():
        entry = obs.get("compared",{}).get(kind,{})
        err = entry.get("error") or {}
        count = entry.get("count",0)
        checks[kind] = {"count":count, "per_frame_max_abs_component":err, "threshold":tol,
                        "all_compared_below_threshold":err["max"] < tol if count and "max" in err else None}
        checks[kind]["over_observed_output"] = ratio(count,observed.get(CLOUD if kind=="all_xyz_m" else ODOM))
    boundary = [
        "Hz=(N-1)/(最后-最初)，只覆盖有两次以上观测的该流窗口；消息头域与观察器回调到达域分列。不同域不得相减。TF按边计，缺少原始雷达/图像header时只显示bag与发送计数。",
        "同步只用整数ns完全相等；最近邻距离仅定位漏配，未用于放宽准入。计数属于观察器订阅视角；比较的每一帧全点按顺序逐分量验证，误差统计对象是每帧最大分量，非所有点的RMSE。",
        "all_lio_cycles=attempts_after_odom+empty_before_odom；双分母另给出源端已发送、观察器收到与节点诊断收到。源端门控首拒原因互斥，any_*可能重叠。一个样本可能触发多个生命周期事件，rejects/seen不是弃帧概率。",
        "processing.csv约每壁钟1秒抽一轮，不能恢复所有LIO/VIO耗时分布。诊断分位数最近至多256次，统计表不是逐帧全程分位数。CPU区间相对单核，RSS全窗口变化含初始化和回放尾部；一次回放的线性斜率不能证明内存泄漏或长稳。",
        "轨迹统计使用各自产生的全部观测；首末净位移、累计弦长与最大相邻变化含场景实际运动及估计变化。跳过无效姿态或非正时间间隔；四元数角度取最短弧。发布速度仅解释适配里程计，原生pose相邻差分使用它自己的发布时钟。缺少独立真值，无法由此推断定位精度、绝对同步误差或实车稳态表现。",
        "样本姿态几何核对限定于isolated_test配置T_imu_base平移[0.2,0,0.1]、单位旋转，TF数值核对仅限保存transform的运行。未保存逐戳误差时PCD不能独立重建对应输出点云。"]
    return {"run":run.name,"scenario":scene,"bag_path":replay.get("path"),"playback_rate":replay.get("rate"),
            "replay_eof":replay.get("eof"),"observer_wall_elapsed_s":obs.get("elapsed_s"),"streams":streams,
            "sync":comparisons,"coverage":info,"observer_checks":checks,"observer_nonfinite":obs.get("nonfinite",{}),
            "source_to_adapter_pose": compare_poses(src,out,[.2,0,.1]),
            "adapter_to_tf_pose": compare_poses(out,tf,[0,0,0]),
            "tf_edges":obs.get("tf",{}),"trajectory":{k:trajectory(v,k==ODOM) for k,v in pose_data.items()
                                                        if " -> " not in k},
            "diagnostics":diagnostics(diag),"resources":resources(res),"processing":processing(proc),
            "sample_index":"samples/index.json" if samples else None, "sample_count":len(samples),"boundaries":boundary}


def main():
    parser = argparse.ArgumentParser(description="统计单次隔离回放，写入 RUN/metrics.json，存在样本时生成 samples/index.json。")
    parser.add_argument("run",type=Path)
    parser.add_argument("--scene",default="未提供",help="来自采集记录的场景标签，例如 静止 或 动态。")
    args = parser.parse_args()
    run = args.run.resolve(strict=True)
    result = summarize(run,args.scene)
    write_json(run/"metrics.json",result)
    print(run/"metrics.json")


if __name__ == "__main__":
    main()
