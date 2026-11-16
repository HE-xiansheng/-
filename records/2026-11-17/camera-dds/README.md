# Gemini 大图 DDS 发布配置候选

三分钟多模块并发 run2 中，独立原生观察器收到2652条NPU预览，但9条源戳不在5975条相机捕获里；Python捕获补证8条，剩1条未定。实际NPU代码复制输入header，不能仅据此断言没有问题。仅对测试相机发布进程应用64MiB FastDDS SHM后，run3 三分钟43/43项通过，相机6024/预览3221个不同源戳完全对应，原生观察器和NPU没有设置该profile。与此前MPP大图现象一致，支持配置发布端容量；不声称逐条历史遗漏原因已证明。

候选给 Gemini 本地启动封装加入 GroupAction 范围的环境设置，加载本驱动 config/fastdds_large_images.xml；不依赖MPP包。仅默认或显式 rmw_fastrtps_cpp 且用户未设置 FASTRTPS_DEFAULT_PROFILES_FILE / FASTDDS_DEFAULT_PROFILES_FILE 时启用。保留UDPv4，其他RMW不加配置。不替换相机SDK/内核驱动/固件，不改变分辨率或相机标定。

run4 使用实际候选启动文件（不再由测试注入profile）与已部署 FAST-LIO。process-identity.json 记录相机容器加载候选XML，观察器/NPU/LIO均无该环境变量；定位二进制7707f011…与分支及并发候选一致。最终通过与否以 ../platform-soak/run4/result.json 为准。

部署仅替换本驱动launch、增加XML，先校验旧HEADd861281、干净工作区、无相机进程及旧文件摘要。部署payload和脚本存本目录，机载不落报告。停止后从 previous-runtime.tar.gz 恢复旧launch并删除新增XML，可只回退本变更；不会回退FAST-LIO或其他模块。正式服务保持原状。

已通过仍只表示此负载组合约三分钟的结果；不代表全部相机帧都被NPU处理，NPU使用最新帧队列，不做30Hz逐帧推理保证。预览源戳必须来自相机是完整性检查，和源图像绝对不丢帧不是同一个结论。实物雷达/云台直连、外参、真实飞控融合与完整任务流程仍待验收。

run4自动配置的180秒运行原始42/43（原生观察首帧之前遗漏2条相机输入）；两条均获Python独立相机源记录证实。两路并集严格复核后43项全通过，保留原始失败，详见../platform-soak/run4/reanalysis.json。run2仍有1条两路均无证据而失败，改写1ns负向检查拒绝。部署机载bfc044a，launch/XML摘要与run4输入完全一致；installed-launch-check.txt确认默认范围设置、显式profile优先、Cyclone不注入，仅构造检查不声称实际运行了其他RMW。当前只有这套已测FastDDS默认场景的运行结果。所有测试进程/临时目录清理，NoMachine active、Git干净。

部署完成 bfc044a，摘要与run4归档候选逐字一致。installed-launch-check.txt确认GroupAction包含环境压栈/恢复；默认添加profile、显式profile和CycloneDDS不添加。该检查只构造launch，不启动设备；实际默认相机容器环境证据见run4/process-identity.json。
