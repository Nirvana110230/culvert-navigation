# culvert_registration

Orange Pi 5 Ultra／ROS 2 Humble 的离线局部配准库与命令行服务。对同一先验三维地图，支持原车两级 PCL ICP 和固定版本 small_gicp 的 GICP。ROS 调用方与文件基准可使用同一 Session 协议。该包自身没有订阅器、TF 发布器或执行器接口。

## 构建与测试

新 C++ 调用者可使用安装后的 `#include <checked_session.hpp>`，在下游 CMake 中执行 `find_package(culvert_registration CONFIG REQUIRED)`，然后将目标链接至 `culvert_registration::culvert_registration_core`。同一 `culvert::CheckedSession` 成功加载地图后，向每条 match 传入调用方预期的 `map_id`、`version`、`sha256`、`generation`、`target_frame`。其余字段与下方 JSONL 示例一致。返回值带后端版本、候选有效性、公共几何与先验门控、子图标识。身份不符时不会进入求解；尚未求解时位姿返回 null。[受检头](src/checked_session.hpp) · [历史测试结果](../../docs/validation.md)

在独立工作区内使用 C++17、PCL 1.12、Eigen3、JsonCpp、OpenSSL Crypto、OpenMP 和私有前缀内的 small_gicp。先设置 `SMALL_GICP_PREFIX` 指向提交 `fa0cfc983417c044b6036f71dfa448038bb9c820` 的已配置／构建／安装前缀。

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$SMALL_GICP_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$SMALL_GICP_PREFIX/lib:${LD_LIBRARY_PATH:-}"
colcon build --packages-select culvert_registration --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
ctest --test-dir build/culvert_registration --output-on-failure
source install/setup.bash
ros2 run culvert_registration registration_server
```

进程保持 stdin 打开后，每行写入一个 JSON 对象，每次会返回一行标准 JSON。先 `load` 取得 generation，再 `match`；更换地图后使用新代次。下例可用 Python 的 json 模块生成一组请求。地图、查询帧、初值和 local 应取自同一次实验定义。示例的两个单位变换表示查询的初值在地图原点，且该时刻 W 与源坐标系重合。

```python
import hashlib, json
from pathlib import Path
from subprocess import Popen, PIPE

p = Popen(["ros2", "run", "culvert_registration", "registration_server"],
          stdin=PIPE, stdout=PIPE, text=True)

def ask(**request):
    p.stdin.write(json.dumps(request, allow_nan=False) + "\n")
    p.stdin.flush()
    return json.loads(p.stdout.readline())

m = Path("map.pcd").resolve()
I = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
meta = ask(op="load", path=str(m), id="example", version="1", frame="map",
           sha256=hashlib.sha256(m.read_bytes()).hexdigest(), units="m",
           gravity="+Z up", origin="fixture origin")
if meta["ok"]:
    result = ask(op="match", scan_path=str(Path("scan.pcd").resolve()), scan_id="sample-01",
                 stamp_ns=1000000000, source_frame="lidar", backend="gicp", threads=1,
                 radius=12, budget_ms=500, initial=I, local=I, generation=meta["generation"])
    print(result["accepted"], result["reason"])
p.stdin.close()
p.wait()
```

C++ 调用方包含 `session.hpp`，为每个地图会话持有一个 `culvert::Session`，串行调用 handle(Json::Value)。共享库／程序链接所需第三方依赖见 CMakeLists。库中保存最近两项局部图缓存，`accepted=false` 的矩阵仅供诊断。带时间门控的调用方传同钟域 now_ns 与 max_age_ms；多假设选择、独立帧确认及 TF 归属由外部调用方管理。[历史测试结果](../../docs/validation.md) · [许可文件](licenses)
