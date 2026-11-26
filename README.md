# 涵洞导航 R1 / R2

本仓保存 ROS 2 Humble 下的同刻观测适配器、隔离 FAST-LIVO2 v3、ICP/GICP 配准工作进程和定位候选节点。R1/R2 的放行范围是离线开发。底盘实际外参、运动限值、独立复访和全局确认仍待实测。正式适配器缺参数时以诊断闭锁，定位节点输出只在测试命名空间。

| 入口 | 用途 |
|---|---|
| [隔离 v3 源码](isolated/v3) 与 [R1 补丁](patches/fast_livo2_culvert_source.patch) | LIO 更新后发布同一测量戳的 IMU 状态与去畸变雷达扫描 |
| [R1 适配器](src/culvert_livo_adapter) | 精确配对、外参变换、速度差分、正式配置闭锁 |
| [R2 配准核心与 worker](src/culvert_registration/README.md) | registration_server 常驻进程，PCL 点到点 ICP 和 small_gicp GICP 共享同一请求协议 |
| [R2 候选节点](src/culvert_localization/README.md) | 模式 A/B、单请求在途、超时和地图换代检查 |
| [离线验证](docs/validation.md) · [逐例统计](benchmarks/r2/results/cases.csv) | 板端历史证据、首次失败与复测、G3 本机核验 |
| [第三方来源](third_party/README.md) · [源文件与版本身份](VERSION-IDENTITIES.json) | 许可文本、冻结依赖、上游入口与 ELF 哈希 |

## 构建、安装和测试

环境为 Ubuntu 22.04、ROS 2 Humble、CMake、C++17、Eigen3、PCL 1.12、OpenSSL、JsonCpp、GNU OpenMP、Python 3 和 NumPy。ROS 开发包由三个 package.xml、CMakeLists.txt 列出，数学测试另需 ament_cmake_gtest。以下命令从克隆根目录运行。每个构建和 ROS 测试写入新目录。安装前缀包含三个 ROS 包和固定的 small_gicp。

```bash
job=$(mktemp -d)
bash scripts/build_navigation.sh "$job/navigation"
bash scripts/test_ros.sh "$job/navigation/install" "$job/ros-tests"
python3 scripts/generate_fixture.py "$job/synthetic"
python3 scripts/verify_evidence.py
```

R1 合成测试使用本机 ROS 域 84，R2 故障注入使用域 92。R2 通过测试替身注入延迟与异常，配准 CTest 对真实数值后端发起请求。安装后可用以下命令定位并调用节点。离线配置文件需要指定地图清单、传感器模式、外部初值和对应的外参。示例中只查询包和工作进程。

```bash
source /opt/ros/humble/setup.bash
export NAV_PREFIX="$job/navigation/install"
export AMENT_PREFIX_PATH="$NAV_PREFIX:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$NAV_PREFIX/lib:${LD_LIBRARY_PATH:-}"
ros2 pkg prefix culvert_localization
test -x "$NAV_PREFIX/lib/culvert_registration/registration_server"
```

## 隔离 v3 与历史回放

v3 的 33 个文件来自板端隔离工作树。其消息包、vikit_common、vikit_ros 和 Sophus 可以使用配套 culvert-fast-livo2 私有仓的 Stage A 源码及依赖环境，系统另需 cv_bridge、image_transport、OpenCV、fmt 和 Boost 等 CMake 所列项。先加载已安装的依赖，再编译本仓已应用补丁的 v3 源；FAST_LIVO_ENV 是可选的私有依赖环境脚本路径。脚本把 ELF 写到指定的新目录。

```bash
bash scripts/build_v3.sh "$job/v3"
git apply --reverse --check --directory=isolated/v3 patches/fast_livo2_culvert_source.patch
```

RK3588 的回放使用 FAST-LIVO2 安装根目录下的 env.sh、runtime 和 configs。将 FAST_LIVO_ROOT、NAV_PREFIX、V3_ELF 指向这三个绝对位置，NAV_DATA_UUID 在操作机私下设置为预先核实的数据盘 UUID。R1 用 `bash scripts/replay.sh candidate /path/to/bag 1` 回放。R2 用 `bash scripts/r2/joint.sh "$run_root" "$fixture" handheld gicp A` 联合回放。run_root 须为已准备的 /data/robot/navigation_dev/R2_*/run_*，fixture 须含经校验的历史数据清单和 PCD。NAV_REPLAY_BAG 可显式指定对应场景的归档袋。脚本缺省检查历史冻结的 v3、适配器 ELF 哈希。针对新构建回放时可分别提供经独立核对的 V3_SHA256 和 ADAPTER_SHA256。各次输出在数据盘新目录；保留板端工作区和历史回放目录供现场读回。

只有 R1 合成夹具可完全从本仓单独生成。复原 R2 的完整 62 例留出集还需外部保存的 R1 运行样本。历史 R2 已核对 86 份 PCD，其中 6 个真实查询为同袋留一。归档目录、逐例 CSV、冻结的运行程序、数据集和执行器身份见证据页。SOURCE-FILES.csv 沿用 G2 候选来源记录；本次新增并核验的文件按各组件证据清单追溯。本仓使用 Git 标签 g2-r2-offline-20261126 标识初始候选快照，标签 g3-nav-r1-r2-20261126 标识本次工程归档。
