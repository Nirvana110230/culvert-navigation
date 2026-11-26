# 离线验证

## 历史 Orange Pi / ARM64

G3 通过 SSH 只读核对原 R1 工作区、R2 的 run_kxRymzMm 和对应数据目录。R1 的 v3 算法 ELF 为 11f84bef…8754f，R2 安装路径的 registration_server 为 912f3051…a8cad。后者与留出实验冻结清单完全相同。同次检查发现构建树路径的 worker 哈希为 ab348730…55053，两个 ELF 区分记录；历史统计绑定安装路径版本。对应源码包含 Session、受检接口、CMake、工作器 server.cpp 和调用端 localization_io.py。[版本清单](../VERSION-IDENTITIES.json) · [板端逐文件来源](evidence/board-package-sources.json)

| 检查 | 板端记录 |
|---|---|
| R1 数学与 ROS 传输 | 11 项 GoogleTest，0 失败。25 项 ROS 断言全部通过，包括缺失实测底盘外参时闭锁 |
| R1 手持回放 | state/scan 各产生 576 帧，同戳配对 576；预热后 odom、定位云和 TF 严格同戳 575 次 |
| R2 worker | 配准协议、调用绑定、无效地图保留旧代次三个 CTest 均通过 |
| R2 ROS 故障 | 初测第 6 个断言在候选时间戳观测处失败；观察端修正后 50/50 通过。保留初次与复测 |
| 同袋手持联合回放 | 各得到 576 个源数据对。ICP/GICP 各提交 114 个请求，各接受 78/111。两个后端各有 1 个超过 500 ms 的请求 |
| 输入异常 | 坏包出现输入停更保护，expected_frontend_stop 为真。完成至正常 EOF 的回放单独统计 |
| 重复负载 | 600.034 s 内 ICP、GICP 各处理 1,200 次请求，跳过周期 0，目标周期 0.5 s |

[摘要与 CTest](evidence/arm64-summary.json) · [R1 断言](evidence/r1-ros.json) · [R1 手持时序](evidence/r1-handheld.json) · [R2 初次结果](evidence/r2-ros-first.json) · [R2 ROS 复测](evidence/r2-ros-retest.json) · [持续负载](evidence/r2-soak.json) · [原始轻量结果指纹](evidence/board-evidence-sha256.json)

## ICP 与 GICP 的同条件留出结果

相同的 62 例中有 53 个应定位正例、9 个应拒负例。合格位姿要求平移误差不超过 0.15 m、姿态误差不超过 3°，并由运行器结合每例应有结果统计。逐例表提供初值扰动场景、接受判定、残差与位姿误差。

| 后端 | 正确接受 | 错误接受 | 正确拒绝负例 | 其他 |
|---|---:|---:|---:|---|
| ICP | 38 | 12 | 9 | 3 例定位失败 |
| GICP | 45 | 2 | 9 | 3 例定位失败，3 例位姿已合格但被拒 |

两后端在相似廊段都出现过相距约 32 m 的错锁。独立实车复访、现场锚点、绝对轨迹真值和底盘实测外参仍是现场验收所需条件。6 个真实查询的同袋留一适合做回归。R2 离线开发保留 ICP 对照，并将 GICP 用于近初值测试。[逐例 CSV](../benchmarks/r2/results/cases.csv) · [冻结条件](evidence/r2-freeze.json) · [历史夹具核验](../benchmarks/r2/results/preflight.json)

## G3 本机复核

在 Ubuntu 22.04 / WSL x86_64 的 ROS 2 Humble 下，G3 用新前缀编译并安装 small_gicp、适配器、配准工作进程和定位节点。适配器 11 个数学用例与 3 个配准 CTest 全部通过。R1 在 localhost 域 84 完成 25/25，R2 在域 92 完成 50/50。首次从 Windows 检出时，CRLF shebang 导致本机 R2 节点无法启动；已固定脚本的 Git 换行并在新目录复测。合成生成器重建 2,713 点地图与 1,423 点扫描，两个 PCD 与 R1 冻结指纹相同。

G2 恢复的正式源在临时目录应用原 R1 补丁后，33 个隔离文件可逐字节还原板端 v3。补丁涉及 IMU_Processing.h/.cpp 与 LIVMapper.h/.cpp 四个文件。部署目录下的原件未变动。用 `python3 scripts/verify_evidence.py` 可在克隆后重算 v3、第三方编译源码、断言和留出统计。[v3 文件清单](evidence/v3-source-sha256.json) · [本机验证结果](evidence/g3-local.json)

同一 WSL 使用已原生构建的 livox_ros_driver2、vikit_common、vikit_ros 和固定的 Sophus 前缀，将隔离 v3 实际编译为 x86_64 fastlivo_mapping。v3 与配准 worker 的 ldd 均可解析全部动态库。[本机 ELF 指纹](evidence/g3-local.json)
