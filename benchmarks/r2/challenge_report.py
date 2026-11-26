"""Summarize the independently seeded supplemental challenge without changing any gate."""
from collections import Counter

from report import fmt, markdown_table


def section(rows):
    singles=[r for r in rows if r['kind']=='single']
    cross=[r for r in rows if r['kind']=='cross_map']
    assert len(singles)==50 and len(cross)==4
    by={(r['case'],r['backend']):r for r in singles}
    summary=[]
    for scene in ['longitudinal_nullspace','narrow_repeat_single','narrow_repeat_compared','uniform_scattered_outliers','similar_maps']:
        for b in ['icp','gicp']:
            sub=[r for r in singles if r['scene']==scene and r['backend']==b]
            summary.append([scene,b.upper(),f'{sum(r["accepted"] for r in sub)}/{len(sub)}',
                            '、'.join(f'{reason}×{n}' for reason,n in Counter(r['reason'] for r in sub).items()),
                            fmt(max(r['wall_ms'] for r in sub))])
    tables=[markdown_table(['条件','后端','放行/请求','决定','最大查询 ms'],summary)]
    choices=[]
    for b in ['icp','gicp']:
        for name in ['site_A','site_B','site_alias_A']:
            r=by[name,b]
            p=r['replies'][0]
            err=r['site_error']
            choices.append([b.upper(),name,r['reason'],fmt(p['rmse']),fmt(p['inlier_ratio']),
                            fmt(err['translation_m']) if err else '—'])
    tables.append(markdown_table(['后端','单图身份','决定','内点 RMSE m','内点比','共同 site 系平移误差 m'],choices))
    detailed=[]
    for name in ['walls_y_-0.5','walls_y_0.5','walls_yaw_-15','walls_yaw_15','seam_single_0','seam_single_1.2','seam_single_2.4',
                 'seam_all','outliers_0.1','outliers_0.3','outliers_0.5']:
        row=[name]
        for b in ['icp','gicp']:
            r=by[name,b]
            err=r['error']
            row.append(f'{r["reason"]} | {fmt(err["long_m"])}, {fmt(err["lat_m"])}, {fmt(err["yaw_deg"])}'.replace('|','·') if err else r['reason'])
        detailed.append(row)
    tables.append(markdown_table(['偏置、窄支涵或离群点','ICP 决定 · 纵 m, 横 m, 航向 °','GICP 决定 · 纵 m, 横 m, 航向 °'],detailed))
    cross_rows=[[r['backend'].upper(),r['case'],r['reason'],r['selected_map'] if r['accepted'] else '无',fmt(r['elapsed_ms']),
                 fmt(r['total_with_cold_loads_ms'])] for r in cross]
    tables.append(markdown_table(['后端','同一次观测的先验图','跨图决定','放行地图记录','匹配耗时之和 ms','计入分别装图 ms'],cross_rows))
    return ['## 独立种子补充挑战',
            '补充集固定种子 12013411，每后端包括 25 个查询及 2 次跨图比较，使用已冻结的二进制及阈值。长廊沿 X 的初值偏差取 −3、−1、−0.5、0、0.5、1、3 m，横向取 ±0.1、±0.5 m，航向取 ±5、±15°。长廊有充分横向平面约束，纵向仍不可观。0.80 m 宽窄支涵每隔 1.2 m 有一组突出接缝，三入口各相隔一个周期。随机离群点在测量空间的盒形区域内均匀分布，注入后占总扫描点的 10%、30%、50%。[补充生成与执行脚本](../../benchmarks/r2/challenge.py) · [逐试次原始返回](evidence/challenge/raw.jsonl)',
            tables[0],tables[2],
            '跨图实验始终使用同一帧、同一源戳与内容散列。人工构造的测量框架 site 给定两条真实不同廊道 A、B 的相对横移 32 m，B 的墙面另加最高 17 mm 的小变形。把每个候选左乘各图的 T_site_map 后，在 site 中按既有判据比较。alias_A 为同一地点 A 的坐标重编码，地图原点相对 site 平移 (6, −2, 1) m。它与 A 使用相同地点标识，并且变换后落在相同物理位置。表内 B 的共同世界误差是合成真值可量化的错廊距离。实车没有经测量确认的 site 变换时，不得把不同地图的局部位置直接相减。',
            tables[1],tables[3],
            '相似错图单独匹配时，两种后端均缺乏可辨识地图身份的外部观测。跨图结果须结合实际参与比较的地图集合和各图的坐标元数据解读。跨图运行预先独立建立目标缓存，表中同时给出纯匹配耗时与计入两次冷装图的成本。这个跨图调度器用于实验，生产 ROS 节点的多先验处理能力以接口实现为准。']
