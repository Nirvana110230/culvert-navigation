# 涵洞导航 · 本地离线候选

本目录交接 culvert_livo_adapter、culvert_localization 和 culvert_registration 的源码、配置、测试与配套许可声明。SOURCE-FILES.csv 给出工作树文件的 SHA256，源码原仓尚无首个提交。原导航索引与对象位于受限介质 orange/navigation.zip。四份 AM 文档的暂存和工作版本在 orange/navigation-am/staged 和 working 目录可独立取得。恢复完整运行材料时使用该归档，其中含冻结的 small_gicp、实验夹具、工作树、R1 与 R2 报告和历史日志。

R1 的条件通过适用于隔离 v3 接口和适配器。R2 已记录 ICP/GICP 的同条件离线比较。62 个留出用例中有 53 个可定位正例，ICP 正确放行 38 个并错误接受 12 个，GICP 正确放行 45 个并错误接受 2 个。实际底盘外参、独立建图与跨次实车复访尚待验证。VERSION-IDENTITIES.json 记录冻结二进制身份、数据来源和报告指纹。

本候选只用于本地交接。第三方依赖源码、原始数据与板端记录通过受限归档交接。适配器和配准包的权属尚待核定，当前禁止上传。
