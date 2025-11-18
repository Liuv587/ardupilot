# AC_ADRC - Active Disturbance Rejection Control Library

## 概述

AC_ADRC 是 ArduPilot 的自抗扰控制（ADRC）算法库实现。ADRC 是一种先进的控制算法，能够自动估计和补偿系统扰动，提供更好的控制性能。

## 主要组件

1. **跟踪微分器（TD - Tracking Differentiator）**：平滑输入信号，避免阶跃输入对系统造成的冲击
2. **扩张观测器（ESO - Extended State Observer）**：观测系统状态和扰动
3. **非线性状态反馈（NLSEF - Nonlinear State Error Feedback）**：PD控制的进阶版本

## 使用方法

### 1. 包含头文件

```cpp
#include <AC_ADRC/AC_ADRC.h>
```

### 2. 创建ADRC控制器实例

```cpp
// 创建ADRC控制器，参数：wo, kp, kd, b
AC_ADRC rate_roll_adrc(10.0f, 0.5f, 0.05f, 1.0f);
```

### 3. 在控制循环中更新

```cpp
// 在控制循环中调用
float dt = AP::scheduler().get_last_loop_time_s();
float output = rate_roll_adrc.update_all(target_rate, actual_rate, dt);
```

## 参数说明

### WO (Observer Bandwidth)
- **描述**：观测器带宽，影响扰动估计的速度
- **范围**：1-100 rad/s
- **调参建议**：
  - 低频扰动：使用较小带宽（5-15）
  - 高频扰动：使用较大带宽（20-50）
  - 过大可能导致抖震

### KP (Proportional Gain)
- **描述**：比例增益，影响系统响应速度
- **范围**：0.01-10.0
- **调参建议**：从较小值开始，逐步增加

### KD (Derivative Gain)
- **描述**：微分增益，影响系统阻尼
- **范围**：0.01-10.0
- **调参建议**：
  - 过大可能导致抖震
  - 抖震时不能通过提高带宽解决，必须减小KD或增大B

### B (Compensation Coefficient)
- **描述**：补偿系数，影响扰动补偿的强度
- **范围**：0.1-10.0
- **调参建议**：
  - 增大B可以减少抖震，但扰动补偿效果会变差
  - 减小B可以增强扰动补偿，但可能导致抖震

## 参数整定策略

1. **固定补偿系数B**，设定较小的KP和KD
2. **尽可能选用大的带宽WO**，观察是否发生抖震
3. **逐步调高KP和KD**
4. **如果控制效果不满意，调整B**，然后重复上述过程

## 集成到姿态控制

参考 `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp`，在 `rate_controller_run()` 函数中：

```cpp
// 替换PID调用
// 原来：
// _rate_roll_pid.update_all(...)

// 改为：
// _rate_roll_adrc.update_all(...)
```

## 注意事项

1. **采样频率**：建议采样频率至少为控制频率的10倍以上
2. **参数调优**：建议先在仿真环境中调优参数
3. **状态重置**：模式切换时调用 `reset()` 重置内部状态
4. **数值稳定性**：代码中已包含状态限制，防止数值发散

## 参考文档

详细算法说明请参考：`ADRC算法介绍.md`

