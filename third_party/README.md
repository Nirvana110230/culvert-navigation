# 组件来源与许可

| 组件 | 来源和版本 | 随仓内容 |
|---|---|---|
| FAST-LIVO2 v3 | 基于项目 Stage A 和 R1 同刻接口补丁，板端 v3 的 33 个源码、头文件及构建/许可文件逐字节核验 | [GPL v2 许可原件](../isolated/v3/src/fast_livo/LICENSE)、[原始包声明](../isolated/v3/src/fast_livo/package.xml)、[补丁附带许可](../patches/LICENSE.fast_livo) |
| small_gicp | 上游 1.0.1，fa0cfc983417c044b6036f71dfa448038bb9c820，Git tree ddfae80300c3fa21fd761ebacc7b8e36659f8aa4 | [用于构建的源码](small_gicp)、[MIT 许可](small_gicp/LICENSE)、[随仓 62 文件指纹](../docs/evidence/small-gicp-build-sources.json) |
| 原 ICP 和内嵌代码 | 原车参考 localizer、nanoflann 与 Sophus 派生代码 | [各组件的原文声明](../src/culvert_registration/licenses) |
| ROS 2 节点组织 | 本项目实现两种输入模式、精确配对、工作进程与候选输出；参考工程提交 042b168110c67aa41ed9e2d9e957d8cae016f0fb | [参考仓固定版本](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/tree/042b168110c67aa41ed9e2d9e957d8cae016f0fb) |

FAST-LIVO2 原文件页眉及 GPL 文本保留，package.xml 声明 BSD，两个声明存在差异。适配器和配准包保留 LicenseRef-Undetermined，定位包声明 Apache-2.0。配套 Livox、vikit 和 Sophus 随其各自项目的条款分发。工程的私有归档授权不改变这些许可。[FAST-LIVO2 上游](https://github.com/hku-mars/FAST-LIVO2)

完整 small_gicp codeload 归档 SHA256 为 5a1eb2b81a21b4715571b4e953fa9c3759a4ae24f6b4a4b7d6c2ef0501ffb935，G2 核对的完整上游树含 153 个文件。G3 从同一归档逐字节提取构建用 CMake 文件、头文件、库实现、许可、上游 README 和文献元数据，未带入上游演示点云。构建脚本启用 OpenMP、关闭可选示例和测试，以系统 Eigen 离线编译。[上游固定提交](https://github.com/koide3/small_gicp/tree/fa0cfc983417c044b6036f71dfa448038bb9c820) · [整树核验记录](../docs/evidence/small-gicp-upstream.json)
