/// @file	AC_ADRC.cpp
/// @brief	Active Disturbance Rejection Control (ADRC) algorithm implementation

#include <AP_Math/AP_Math.h>
#include "AC_ADRC.h"

// Parameter table
const AP_Param::GroupInfo AC_ADRC::var_info[] = {
    // @Param: WO
    // @DisplayName: ADRC Observer Bandwidth
    // @Description: Observer bandwidth (ωo) for Extended State Observer (ESO). Higher values provide faster disturbance estimation but may cause oscillation.
    // @Units: rad/s
    // @Range: 1 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("WO", 0, AC_ADRC, _wo, default_wo),

    // @Param: KP
    // @DisplayName: ADRC Proportional Gain
    // @Description: Proportional gain (kp) for PD controller in ADRC
    // @Range: 0.01 10.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("KP", 1, AC_ADRC, _kp, default_kp),

    // @Param: KD
    // @DisplayName: ADRC Derivative Gain
    // @Description: Derivative gain (kd) for PD controller in ADRC
    // @Range: 0.01 10.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("KD", 2, AC_ADRC, _kd, default_kd),

    // @Param: B
    // @DisplayName: ADRC Compensation Coefficient
    // @Description: Compensation coefficient (b) for disturbance compensation. Larger values reduce disturbance compensation but decrease oscillation.
    // @Range: 0.1 10.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("B", 3, AC_ADRC, _b, default_b),

    // @Param: FF
    // @DisplayName: ADRC FeedForward Gain
    // @Description: FeedForward gain (kff) for ADRC. Produces an output value that is proportional to the target rate. Higher values provide faster response to target changes. For yaw axis, typical values are 0.3-2.0. Values above 1.0 may be needed for fast yaw response matching PID performance.
    // @Range: 0.0 5.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF", 4, AC_ADRC, _kff, default_kff),

    AP_GROUPEND
};

// Constructor
AC_ADRC::AC_ADRC(float initial_wo, float initial_kp, float initial_kd, float initial_b, float initial_kff) :
    default_wo(initial_wo),
    default_kp(initial_kp),
    default_kd(initial_kd),
    default_b(initial_b),
    default_kff(initial_kff)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // initialize states
    reset();

    // update observer gains based on initial wo
    update_observer_gains();

    // initialize info structure
    memset(&_adrc_info, 0, sizeof(_adrc_info));
    _target = 0.0f;
}

// reset - reset all internal states
void AC_ADRC::reset()
{
    _z1 = 0.0f;
    _z2 = 0.0f;
    _z3 = 0.0f;
    _v1 = 0.0f;
    _v2 = 0.0f;
    _u0 = 0.0f;
    _disturbance_compensation = 0.0f;
    _target = 0.0f;
}

// update_observer_gains - update ESO gains based on observer bandwidth
void AC_ADRC::update_observer_gains()
{
    const float wo_val = _wo.get();
    _beta01 = 3.0f * wo_val;
    _beta02 = 3.0f * wo_val * wo_val;
    _beta03 = wo_val * wo_val * wo_val;
}

// tracking_differentiator - 跟踪微分器 (TD)
// 简化实现：使用一阶滤波器作为TD
void AC_ADRC::tracking_differentiator(float target, float dt)
{
    if (!is_positive(dt)) {
        return;
    }

    // 简化TD实现：使用一阶低通滤波器
    // v1 跟踪目标值，v2 跟踪目标值的导数
    const float alpha = 1.0f - expf(-dt * _wo.get());  // 自适应时间常数
    _v1 = _v1 + alpha * (target - _v1);
    
    // v2 是 v1 的导数
    if (is_positive(dt)) {
        _v2 = (target - _v1) / dt;
        // 限制v2的变化率，避免噪声放大
        _v2 = constrain_float(_v2, -1000.0f, 1000.0f);
    }
}

// extended_state_observer - 扩张观测器 (ESO)
// 根据公式：
// e = z1 - y
// z1(k+1) = z1(k) + h * (z2(k) - β01*e)
// z2(k+1) = z2(k) + h * (z3(k) - β02*e + b*u(k))
// z3(k+1) = z3(k) - h * β03*e
void AC_ADRC::extended_state_observer(float measurement, float control_input, float dt)
{
    if (!is_positive(dt)) {
        return;
    }

    // 计算观测误差
    const float e = _z1 - measurement;

    // 更新观测器状态（使用欧拉法）
    _z1 = _z1 + dt * (_z2 - _beta01 * e);
    
    // Anti-windup logic for ESO: if control input is saturated, stop integrating disturbance
    // This prevents z3 from winding up when the actuators cannot deliver the requested torque
    float z3_correction = _z3 - dt * _beta03 * e;
    
    // Check if control input was saturated in the previous step
    // We assume saturation limit is roughly +/- 1.0 (normalized input)
    // If saturated and the error is driving it further into saturation, stop z3 update
    const float saturation_limit = 1.0f;
    bool saturated = (fabsf(control_input) >= saturation_limit);
    
    // Simple anti-windup: if saturated, only allow z3 to reduce magnitude
    if (saturated) {
        // If saturated positive and z3 is trying to increase (become more positive), block it
        // Note: z3 definition is z3_dot = -beta03 * e.
        // So we look at the sign of (-beta03 * e) vs sign of control_input
        // Actually, simplest is just to clamp z3 or reduce its gain when saturated
        // Let's just clamp z3 to a reasonable range relative to b * max_u
        float max_disturbance = _b.get() * 2.0f; // allow some overhead
        z3_correction = constrain_float(z3_correction, -max_disturbance, max_disturbance);
    }
    
    _z2 = _z2 + dt * (z3_correction - _beta02 * e + _b.get() * control_input);
    _z3 = z3_correction;

    // 防止状态发散（可选的安全措施）
    _z1 = constrain_float(_z1, -10000.0f, 10000.0f);
    _z2 = constrain_float(_z2, -1000.0f, 1000.0f);
    _z3 = constrain_float(_z3, -10000.0f, 10000.0f);
}

// nonlinear_state_error_feedback - 非线性状态误差反馈 (NLSEF)
// 根据公式：
// e1 = v1 - z1
// e2 = v2 - z2
// u0 = kp*e1 + kd*e2
// u = u0 - z3/b
float AC_ADRC::nonlinear_state_error_feedback(float dt)
{
    // 此函数不再被 update_all 调用，逻辑已移至 update_all 中以访问 measurement
    return 0.0f;
}

// update_all - main update function
float AC_ADRC::update_all(float target, float measurement, float dt)
{
    // 检查输入有效性
    if (!isfinite(target) || !isfinite(measurement) || !isfinite(dt)) {
        return 0.0f;
    }

    if (!is_positive(dt)) {
        return 0.0f;
    }

    // 更新观测器增益（如果wo参数改变了）
    // 注意：这里每次更新可能效率不高，可以考虑在参数改变时更新
    update_observer_gains();

    // 保存目标值
    _target = target;

    // Step 1: 跟踪微分器 (TD)
    tracking_differentiator(target, dt);

    // Step 2: 扩张观测器 (ESO)
    // 注意：这里需要先使用上一次的控制输入
    // 为了简化，我们使用当前的控制输出（会有延迟，但影响较小）
    extended_state_observer(measurement, _u0, dt);

    // Step 3: 非线性状态误差反馈 (NLSEF)
    // float output = nonlinear_state_error_feedback(dt); // 修改调用方式，传入measurement
    
    // 我们需要修改 nonlinear_state_error_feedback 内部实现，或者直接在这里计算
    // 为了不改变头文件，我们修改 nonlinear_state_error_feedback 函数体
    
    // 计算误差
    // CRITICAL CHANGE: 使用 (target - measurement) 直接计算误差
    // 这避免了观测器 z1 的滞后或超调影响 P 项的响应
    // 这使得 P 项的行为完全等同于标准 PID 的 P 项
    const float e1 = target - measurement;  
    const float e2 = _v2 - _z2;  // 速度误差：v2(TD导数) - z2(ESO导数)

    // PD控制律
    // 注意：这里我们假设 b 是扰动补偿增益，而 kp/kd 是控制器增益
    // 在标准LADRC中，u = (kp*e - z3)/b0
    // 这里我们采用 u = kp*e + kd*e2 - z3/b0 的形式
    _u0 = _kp.get() * e1 + _kd.get() * e2;

    // 扰动补偿
    const float b_val = _b.get();
    if (is_zero(b_val)) {
        _disturbance_compensation = 0.0f;
    } else {
        _disturbance_compensation = _z3 / b_val;
    }

    // 最终控制输出 (不包含前馈)
    float output = _u0 - _disturbance_compensation;

    // 计算前馈项
    const float feedforward = target * _kff.get();
    
    // 更新信息结构（用于日志记录）
    _adrc_info.target = target;
    _adrc_info.actual = measurement;
    _adrc_info.error = e1;
    _adrc_info.P = output;
    _adrc_info.D = 0; 
    _adrc_info.FF = feedforward;

    // 最终输出 = 状态反馈控制量 + 前馈
    _u0 = output + feedforward;
    
    // Output limitation (Saturation)
    // Limit output to standard normalized range [-1, 1]
    // This is critical for ESO stability (it needs to know the ACTUAL applied control)
    _u0 = constrain_float(_u0, -1.0f, 1.0f);
    
    return _u0; 
}

