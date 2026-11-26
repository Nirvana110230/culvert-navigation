"""Render the human-readable report from frozen board measurements and the analysis tables."""
import argparse
import json
from pathlib import Path

import numpy as np

from common import read_json


def markdown_table(headers, rows):
    return '\n'.join('| ' + ' | '.join(map(str,row))+' |' for row in [headers,['---']*len(headers),*rows])+'\n'


def fmt(v):
    return '—' if v is None else f'{v:.4g}'


def quantiles(values, keys=('p50','p95','max')):
    return ' / '.join(fmt(values[k]) for k in keys)


def read_raw(path):
    return [json.loads(s) for s in Path(path).read_text(encoding='utf-8').splitlines()]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--summary',type=Path,required=True)
    parser.add_argument('--raw',type=Path,nargs='+',required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--challenge',type=Path)
    a=parser.parse_args()
    d=read_json(a.summary)
    raw=[r for file in a.raw for r in read_raw(file)]
    groups=d['groups']
    g=[x for x in groups if 'mode' not in x and 'scene' not in x]
    hold=[x for x in g if x['split']=='holdout']
    body=['# ICP 与 GICP 对照实验报告',
          '2026-11-25，Asia/Shanghai。实测设备为 Orange Pi 5 Ultra / RK3588 / aarch64。开发分析使用 development2 的 252 条，冻结评估使用 evaluation 的 544 条。其中共 388 条精度与历史记录、360 条冷热测时、48 条逐出访问记录。板端 UTC 起始记录以 environment.json 原值存档，两机日历有偏差。时间分位数来自本机单调时钟。']
    body+=['## 留出结果与选型',
           '**在本轮合成留出集上，GICP 比 ICP 多正确放行 7 个正例、少误受 10 次。GICP 在 2 m 初值偏差下仍两次接受接近 2 m 的错误位姿。** 单机近初值查询可优先继续验证 GICP；两种后端单独使用都不足以放行无人导航。',
           markdown_table(['集合/后端','正 / 负例','正确放行 / 正例','应拒负例中正确拒绝','错误接受 / 不合格','正确位姿被拒 / 有正确位姿','未收敛 / 全部','总时延超 500 ms'],
                          [[x['split']+'/'+x['backend'].upper(),f'{x["n_expected_accept"]} / {x["n"]-x["n_expected_accept"]}',
                            f'{x["correct_accepted"]}/{x["n_expected_accept"]}',
                            x['correct_rejected'],f'{x["false_accepted"]}/{x["n_unsuitable"]}',
                            f'{x["correct_pose_rejected"]}/{x["n_pose_qualified"]}',f'{x["not_converged"]}/{x["n"]}',x['over500']]
                           for x in g if x['split'] in ('debug','tune','holdout')]),
           '留出集 53 个可定位查询中，ICP 正确放行 71.7%，GICP 为 84.9%。两者均拒绝全部 9 个应拒负例，遗漏的错误来自可定位场景中的错误局部解。无效输入也会记录为未收敛。所有场景的分母、收敛、残差、误差及拒绝原因都可从原始返回复算。[逐例 CSV](../../benchmarks/r2/results/cases.csv) · [全分组 CSV](../../benchmarks/r2/results/accuracy_groups.csv)']
    body+=['## 初值范围与误差',
           '每格对相同条件的三次扫描/方向试验统计，单元顺序为「正确放行、未收敛、正确位姿被拒、错误接受」。未收敛与其他事件可同时发生。其余未放行试次的原因见逐例表。']
    bins={(x['backend'],x['mode'],x['bin_m'],x['bin_deg']):x for x in groups if 'mode' in x and x['split']=='holdout'}
    rr=[]
    for m,angle in [(.1,2),(.3,5),(.5,10),(1,20),(2,40)]:
        cells=[f'{m:g} / {angle}']
        for mode,dist,deg in [('translation',m,0),('rotation',0,angle),('combined',m,angle)]:
            for b in ['icp','gicp']:
                x=bins[(b,mode,dist,deg)]
                cells.append('、'.join(str(x[k]) for k in ['correct_accepted','not_converged','correct_pose_rejected','false_accepted']))
        rr.append(cells)
    body+=[markdown_table(['档位 m / °','纯平移 ICP','纯平移 GICP','纯旋转 ICP','纯旋转 GICP','混合 ICP','混合 GICP'],rr),
           '下表统计留出集有已知真值且输出数值位姿的试次，包含接受与拒绝，分布表示绝对误差 P50 / P95 / 最大值。包含两条重复段的单候选结果，所选入口可产生约 24 m 的误差，整组已拒绝。只看已放行位姿的同口径统计另列。',
           markdown_table(['度量','ICP 全部有值','GICP 全部有值','ICP 已放行','GICP 已放行'],
                          [[title,*[quantiles(x[prefix+k]) for prefix in ['abs_','accepted_abs_'] for x in hold]]
                           for k,title in [('translation_m','三维平移 m'),('rotation_deg','三维旋转 °'),('long_m','纵向 m'),
                                           ('lat_m','横向 m'),('vert_m','竖向 m'),('roll_deg','横滚 °'),('pitch_deg','俯仰 °'),('yaw_deg','航向 °')]]),
           '![初值与单线程热时延对照](../../benchmarks/r2/results/accuracy_and_time.png)',
           '留出集 GICP 的两次错误接受共享第 2 个扫描和偏置方向。初值分别仅平移 2 m，以及平移 2 m 并旋转 40°。最终平移误差为 1.977、2.009 m，旋转误差仅 0.163、0.204°。前一例内点 RMSE 为 0.136 m、内点比 0.826、弱特征值比 0.0281，所有单候选质量门均通过。ICP 最严重的错误接受为混合 2 m / 40° 的 2.004 m 与 7.930°。GICP 的三次正确解被过修正门拒绝，发生在 2 m 或 40° 的边界，最终平移误差均不足 0.6 mm。这是超出静态修正幅度门的代价。']
    body+=['## 已知真值、退化与同袋复核',
           'R1 原样非对称地图的精确先验和有偏先验均通过。两次 ICP 三维平移误差为 0.1255、0.1252 mm，旋转为 0.00243°；GICP 为 0.0192、0.0182 mm 与 0.0000432、0.0000472°。该样例的地图点与扫描点一一对应，误差大小仅作实现回归。',
           markdown_table(['留出场景','ICP 接受/请求','GICP 接受/请求','ICP 内点 RMSE P50 m','GICP 内点 RMSE P50 m'],
                          [[scene,*[f'{x["accepted"]}/{x["n"]}' for x in pair],*[fmt(x['rmse']['p50']) for x in pair]]
                           for scene in ['identity','translation','rotation','mixed','reverse','noise','dynamic','occlusion','sparse','empty','nonfinite','wrong_map','parallel_walls','repeat_exact','repeat_near']
                           for pair in [[next(x for x in groups if x.get('scene')==scene and x['split']=='holdout' and x['backend']==b) for b in ['icp','gicp']]]]),
           '留出长廊三个轴向先验均由 weak_geometry 拒绝，两种后端的最弱方向 X 分量绝对值均超过 0.99999。重复段的两个相距 24 m 的候选在每一项均单独通过。精确重复的 ICP 两候选 RMSE 为 0.110074、0.110073 m，GICP 两者均约 0.109692 m。近似重复 GICP 为 0.110060 和 0.109692 m，候选组拒绝。此机制以两处入口均已有候选为前提。普通错图由不同形状区域构造，结论不能外推到几何相近的地图。映射至 XY 与 XZ 的颜色分别代表目标与各候选点云。',
           '![合成与历史点云投影](../../benchmarks/r2/results/registration_overlays.png)',
           '冻结历史六帧对两种后端均为 6/6 接受。表中的平移、角度为相对同次 FAST-LIVO2 初值的变动。没有现场独立真值。',
           markdown_table(['查询','ICP 内部变动 mm / °','GICP 内部变动 mm / °','ICP / GICP 内点 RMSE m'],
                          [[name,*[f'{1000*x["reference_delta"]["translation_m"]:.2f} / {x["reference_delta"]["rotation_deg"]:.3f}' for x in pair],
                            ' / '.join(f'{x["replies"][0]["rmse"]:.4f}' for x in pair)]
                           for name in ['static_q0','static_q1','static_q2','handheld_q0','handheld_q1','handheld_q2']
                           for pair in [[next(x for x in raw if x['case']==name and x['backend']==b) for b in ['icp','gicp']]]])]
    body+=['## 单后端性能、线程与缓存',
           '三个地图依次包含 9,100、45,500、145,600 个原始点。大图把同一几何平移到局部窗口之外，始终向同一入口输入 3,360 点。这样可以观察全图装载开销，局部查询密度保持一致。每种地图、后端和请求线程均新开 2 个进程，每个进程先装图并做 1 次冷匹配，再做 9 次热匹配。冷指新进程及空的应用缓存，操作系统页缓存未清除。P99 为小样本插值，不能作为长期尾时延。CPU% 以一个核心为 100%。',
           '表内的请求线程为 1/2/4，PCL ICP 应答的有效线程和进程线程均为 1；GICP 应答和热运行的进程观察值分别为 1/2/4。两类算法的请求参数均完整遍历。下表列热总时延、冷总时延与加载，各数值均为 ms。[全部 36 个冷热分组、预处理、匹配、CPU、RSS](../../benchmarks/r2/results/performance.csv)']
    p=d['performance']
    pr=[]
    for x in p:
        if x['cache']!='hot': continue
        cold=next(y for y in p if y['cache']=='cold' and all(y[k]==x[k] for k in ['case','backend','threads']))
        pr.append([x['map_points'],x['backend'].upper(),f'{x["threads"]}/{x["threads_effective"]}',fmt(cold['load_ms']['p50']),
                   fmt(cold['wall_ms']['p50']),quantiles(x['wall_ms'],('p50','p95','p99','max')),
                   fmt(x['cpu_pct_one_core']['p50']),fmt(x['peak_rss_mib']),x['over500']+cold['over500']])
    body+=[markdown_table(['地图点数','后端','请求/有效','装图 P50','冷匹配 P50','热 P50 / P95 / P99 / 最大','热 CPU% P50','峰值 RSS MiB','超500 ms /20'],pr),
           '上述 360 个冷、热单次匹配均在 500 ms 内。当前规模中，GICP 4 线程在 145,600 点图热缓存的 P95 为 20.0 ms，PCL ICP 单线程为 83.6 ms。CPU、峰值 RSS、目标预处理与现场 FAST-LIVO2 争用需要连同联合回放判断。分阶段已有 scan_read_ms、子图提取、GICP 源与目标预处理、对齐，以及统一几何评估。ICP 各阶段提供包括源降采样、迭代及原生 fitness 在内的整段耗时。',
           '5 与 16 块地图中的 A→B→C→A 请求，分别对应中心沿纵向移动 0、32、64、0 m。所有 12 组访问的缓存命中依次为 真/否/否/否，两项容量的累计逐出数为 0/0/1/2，最后重新访问 A 时发生重建。目标入口局部细图有 7,147 点。空间上扩展同样的场景不代表真实大图的几何复杂度或独立地形。[逐出逐步记录](../../benchmarks/r2/results/evictions.csv)',
           '基准原始返回含各次两级 ICP fitness、迭代数、GICP 原生代价和 Hessian、点数、残差分位数、六维与平面弱方向、修正量、地图版本和缓存代次。不同后端的原生代价具有不同量纲。比较依照共同的米制几何度量与独立已知真值。[开发集原始记录](evidence/development2/raw.jsonl) · [冻结评估原始记录](evidence/evaluation/raw.jsonl) · [冻结参数与 SHA256](evidence/evaluation/freeze.json) · [可复算结果](../../benchmarks/r2/results/summary.json)']
    if a.challenge:
        from challenge_report import section
        body.extend(section(read_raw(a.challenge)))
    a.out.write_text('\n\n'.join(body)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
