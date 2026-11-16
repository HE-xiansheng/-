# 风扇PWM与短暂CPU限频：只读补充核对

600秒测试记录了policy6上限短暂下降，但当时没有同步风扇/cooling_device状态。本阶段读取现有sysfs、设备树及服务，不重新跑负载，不写PWM，不启停服务，不更换驱动/内核。

初次读数：.224约50.846°C时PWM50；.235约77.615°C时PWM255。继续核对发现两机设备树的rockchip,temp-trips、cooling-levels和pwms逐字节解码值一致，PWM即时值不能作为配置差异的证据。新机自然降至47.153°C时PWM为0，旧机77.615°C仍255。

| 温度阈值 | 驱动state | cooling-level/PWM |
| --- | --- | --- |
| 50°C以下 | 0 | 0 |
| ≥50°C | 1 | 50 |
| ≥55°C | 2 | 100 |
| ≥60°C | 3 | 150 |
| ≥65°C | 4 | 200 |
| ≥70°C | 5 | 255 |

香橙派官方BSP分支的pwm-fan.c明确通过rockchip_system_monitor温度通知读取这些阈值并调整PWM：MAX_PWM255、pwm_fan_temp_to_state、pwm_fan_thermal_notifier_call和pwm_fan_register_thermal_notifier。使用这一通知路径时驱动不再注册通常的thermal cooling_device，因此/sys/class/thermal没有单独的pwm-fan cdev不能直接解释为自动风扇控制缺失；pwm-fan.service为disabled/inactive也不能说明内核风扇控制未运行。先前源文件中pwm-fan.sh启动引用属于其他板型分支，5 Ultra没有该脚本，不将其当作故障。

官方源码归档vendor-pwm-fan.c，分支提交eb1c681e5184e51d8ce1f351559d149d17f48b57，URL/摘要见official-source.json。这是机制参考，不证明已安装5.10.160内核与当前官方分支二进制逐字相同。运行时风扇hwmon没有fan1_input，PWM值不证明物理转速或风扇实际接线。

新机soc-thermal的75/85°C为passive、115°C为critical，迟滞2°C；三组CPU和一组devfreq cooling device绑定trip_index1，即85°C，收尾时cur_state均0。它们的事后状态不能回填为600秒测试时的状态，也不能单独证明上限短暂下降的原因。

## 结论与后续

没有证据需要复制固定PWM255服务或替换风扇驱动；新旧设备树控制配置一致。本轮未作风扇配置修改，因此无回退动作。上轮短暂频率限制和长期热稳定性仍待查，后续真正需要的实物/持续验证将同步记录fan_pwm、cooling_devices与温度/CPU频率，避免只凭事后读数推断。

测试工具resource()已增加这两类只读采样并通过Python语法检查，尚未以新采样版本重跑全负载；上轮600秒原始结果保持不变。没有为给新字段制造通过记录而重复整轮测试。

## 当前外部依赖

本次仍观察到.224雷达网口NO-CARRIER、无云台USB网卡，仅飞控与Gemini在位。600秒测试及归档取回均已结束，没有需要继续等待的后台测试。全套实物数据/定位/真实融合及任务链验收需要接入缺失设备；网络候选与正式服务切换按原约束确认后执行。此结论不缩小完整平台迁移目标，也不将其标记完成。
