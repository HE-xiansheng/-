# 当前部署源码与GitHub逐文件核对

2026-11-17 10:57只读读取.224实际Git提交d896200ce57edba07616a6c942a76200a83071ee及全部git ls-files。1058个普通文件与电脑/GitHub platform目录SHA256逐个一致，无缺失、无内容差异、无缺失/其他类型跟踪项。两个符号链接config→03_tools/ros2_migration/config、humble_ws→02_drivers/sensors/workspace单独记录，没有在电脑展开绝对路径别名。

这证明当前机载已跟踪源码均有相同归档，不证明未跟踪安装二进制、硬件采集、外参或飞控融合验收。没有写机载、重新构建、改服务或安装任何候选。

同次现场检查.224雷达口carrier=0、仅板载网口/WiFi/lo，Gemini335L与PX4 USB在位，无所属LIO/转发测试进程。旧机.235仍可SSH、Safina_sensor.service active；其平台根目录无.git，不能以git status检查旧机改动，因此本次不宣称旧机根目录Git干净。

上轮为实际进展（分段实采与直连对照严格失败），本轮补齐当前部署全跟踪源码的一致性证据。现仍缺新机雷达/云台实物链路、外参和实际PX4接收融合，不以重复无线失败或仿真替代。报告只存电脑/GitHub。
