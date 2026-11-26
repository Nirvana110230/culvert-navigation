# 离线定位候选节点

ROS 2 Humble 包 `culvert_localization` 以整数纳秒严格配对一帧扫描和一帧位姿，调用常驻 `culvert_registration/registration_server`。配准后端、加载的地图和外部先验决定候选结果的含义。独立话题下的有效候选只对应这一次观测。

## 输入坐标和时间

| 模式 | 点云与位姿 | 变换 |
|---|---|---|
| A | `/culvert_source/scan` 为 `livo_lidar`，`/culvert_source/state` 为 `camera_init → imu` | 扫描点保留雷达坐标，`T_W_L = T_W_I × T_I_L`。输入初值为 `T_map_L`，输出修正为 `T_map_L × inverse(T_W_L)`。|
| B | `/culvert_test/localization_cloud` 为 `test_base_link`，`/culvert_test/odom` 为 `test_odom → test_base_link` | 直接使用 `T_test_odom_base`，输入初值为 `T_map_base`，输出修正为 `T_map_base × inverse(T_test_odom_base)`。|

R1 在 LIO 校正后生成同一个整数戳并复制给两条消息，扫描已在相应时刻去畸变；B 的适配器已经施加雷达到底盘的变换。A 模式的 `imu_from_lidar_json` 须与当次录制和候选算法配置核对。B 模式采用的底盘外参来自 R1 测试配置，不能据此推算实车轴心。[R1 源补丁](../../patches/fast_livo2_culvert_source.patch) · [R1 适配器](../culvert_livo_adapter/src/adapter.cpp) · [R1 测试外参](../culvert_livo_adapter/config/isolated_test.yaml)

测量时钟取 R1 播放器发布的 `/clock`，默认最大测量年龄 0.75 s、前瞻容差 0.10 s。单调壁钟另行约束 0.25 s 配对期限、0.75 s 输入停更及请求总预算。暂停会清空等待项并撤销在途结果，回退锁存至重启。在一段时钟内每种输入各有独立高水位，重复与倒序的扫描均不能再次提交。32 位秒和纳秒均以整数读写。[测量时钟播放器](../../scripts/play_measurement_clock.py)

## 配置和运行

从 `config/offline.yaml` 复制每次运行的配置。默认地图、初值和外参均留空。A/B 可分别指定 `backend: icp` 或 `gicp`。默认匹配上限 2 Hz，一个请求在途，最多保留一个最新完整帧对和八个未配对时间戳。配对回调校验布局、大小、坐标及可选强度的有限性。超过 12,000 点的扫描按原始次序等步长采样，JSONL 同时写入输入与提交点数。

| 参数 | 内容 |
|---|---|
| `map_manifest` | 地图 JSON 文件路径；必填 `path,id,version,sha256,frame,units,gravity,origin`，所有值均为非空字符串。PCD 相对路径按清单所在目录解析，`units` 取 `m`，SHA256 对应实际 PCD 字节。后端检查哈希并返回装载代次。|
| `initial_pose_json` | 字符串形式的行主序 4×4 矩阵，即本次地图加载后第一个合格完整帧处的地图先验。也支持含 `map_id,version,sha256,matrix` 的 JSON 对象绑定地图。节点以同帧的本地变换求 `T_map_W`，之后用该修正和各自本地位姿预测。只有接受的新匹配能更新修正。|
| `alternative_poses_json` | 额外候选矩阵数组，最多八项。与主先验明显不同的外部候选触发 `ambiguous_initial_hypotheses` 并闭锁。实际多假设几何评分的调参和留出结果见基准实验。|
| `imu_from_lidar_json` | 模式 A 的 4×4 刚体变换，模式 B 忽略该参数。所有矩阵均检查齐次末行、旋转正交性、行列式和有限值。|
| `budget_ms`、`match_hz`、`threads`、`radius` | 节点总预算从最早一条配对输入到结果提交计算。排队占用先扣除，剩余预算传给后端。线程数 1–8，局部子图半径单位米。配置包默认 350 ms；实际比较以每次保存的 YAML 为准。|
| `record_path`、`save_inputs` | 前者为本次 JSONL 新文件，父目录须已存在；后者可选，启用后新建同名 `.inputs` 目录，在工作线程内为每个提交的扫描写一个 NPZ。|

```bash
source /opt/ros/humble/setup.bash
source /path/to/round2/install/setup.bash
export ROS_LOCALHOST_ONLY=1
# 使用与隔离测量时钟、源节点一致的专用域。
ros2 launch culvert_localization localization.launch.py config:=/path/to/run/params.yaml
# 直接运行同一节点
ros2 run culvert_localization localizer --ros-args --params-file /path/to/run/params.yaml
```

运行中可修改 `map_manifest`、`initial_pose_json` 和 `alternative_poses_json`，随后调用 `/culvert_test/localization/reload_map`（`std_srvs/srv/Trigger`）一起生效。收到切换请求即关闭输出，待工作进程完成旧请求并验证新地图后开启新代。失败会保持闭锁。更换地图后继续使用相同的未绑定初值会得到 `initial_map_mismatch`；应提供新图对应的先验。后端进程异常退出也需显式重载。超时请求允许后端自然计算完毕，调度线程继续接收输入，记录超时并丢弃迟到结果。关闭节点时先结束工作队列，向进程发送 EOF，异常不退出的子进程才进行清理。

## 输出和证据

所有端点均在 `/culvert_test/localization/` 下。成功时发布 `candidate`（`PoseStamped`，地图中源帧位姿）和 `correction`（`TransformStamped`，地图到本地世界系的修正）。两者的戳均为扫描测量时刻。节点不广播 TF。`diagnostics` 为 5 Hz 单调壁钟驱动的 `DiagnosticArray`，含最近接受戳和双时钟年龄、队列、计数、时钟状态、候选有效位和固定为假的导航许可。

`decision`（`std_msgs/String`）及 `record_path` 写同一组 JSON 行。按 `kind` 区分输入拒绝、配对淘汰、装图、起始锚点、提交、超时、终态裁决和工作进程异常。切图后的迟到结果另写 `drop/late_result`，含原请求代次。对于已提交帧，以 `epoch + stamp_ns` 关联证据，用 `input_path` 相对 JSONL 所在目录打开 NPZ。归档含实际发给后端的 `points[N,3]`、`initial`、`local`、源帧名、地图 ID/版本/哈希/坐标系和后端代次。JSONL 的 `result` 原样保留后端的原生指标、公共指标、退化、资源和阶段耗时；`end_to_end_ms` 包含输入配对、排队、序列化、后端处理与定时器收取结果。后端回复头与请求的扫描戳、ID、地图、代次、源帧及算法必须一致，两矩阵还需满足闭环等式。缺失初值、错图、超时或退化的帧只产生诊断和拒绝记录。
