#pragma once

/// @file	AC_ADRC.h
/// @brief	Active Disturbance Rejection Control (ADRC) algorithm
/// @description	自抗扰控制算法实现，包含跟踪微分器(TD)、扩张观测器(ESO)和非线性状态反馈(NLSEF)

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <stdlib.h>
#include <cmath>
#include <AC_PID/AP_PIDInfo.h>

/// @class	AC_ADRC
/// @brief	ADRC control class for ArduPilot
class AC_ADRC {
public:

    struct Defaults {
        float wo;      // 观测器带宽 (observer bandwidth)
        float kp;      // 比例增益 (proportional gain)
        float kd;      // 微分增益 (derivative gain)
        float b;       // 补偿系数 (compensation coefficient)
    };

    // Constructor for ADRC
    AC_ADRC(float initial_wo, float initial_kp, float initial_kd, float initial_b);
    AC_ADRC(const AC_ADRC::Defaults &defaults) :
        AC_ADRC(
            defaults.wo,
            defaults.kp,
            defaults.kd,
            defaults.b
        ) { }

    CLASS_NO_COPY(AC_ADRC);

    /// update_all - set target and measured inputs to ADRC controller and calculate outputs
    /// @param target: 目标值 (desired value)
    /// @param measurement: 测量值 (measured value)
    /// @param dt: 采样周期 (sampling period in seconds)
    /// @return 控制输出 (control output)
    float update_all(float target, float measurement, float dt);

    /// reset - reset all internal states
    void reset();

    /// get accessors for parameters
    const AP_Float &wo() const { return _wo; }
    AP_Float &wo() { return _wo; }
    const AP_Float &kP() const { return _kp; }
    AP_Float &kP() { return _kp; }
    const AP_Float &kD() const { return _kd; }
    AP_Float &kD() { return _kd; }
    const AP_Float &b() const { return _b; }
    AP_Float &b() { return _b; }

    /// set accessors for parameters
    void set_wo(const float v) { _wo.set(v); update_observer_gains(); }
    void set_kP(const float v) { _kp.set(v); }
    void set_kD(const float v) { _kd.set(v); }
    void set_b(const float v) { _b.set(v); }

    /// get internal states (for logging/debugging)
    float get_z1() const { return _z1; }  // 输出估计值
    float get_z2() const { return _z2; }  // 输出导数估计值
    float get_z3() const { return _z3; }  // 扰动估计值
    float get_v1() const { return _v1; }  // TD输出1
    float get_v2() const { return _v2; }  // TD输出2

    /// get control components (for logging/debugging)
    float get_u0() const { return _u0; }  // PD控制输出
    float get_disturbance_compensation() const { return _disturbance_compensation; }  // 扰动补偿量

    /// set target and actual values (for logging purposes)
    void set_target(float target) { _adrc_info.target = target; }
    void set_actual(float actual) { _adrc_info.actual = actual; }

    /// get info structure (for logging)
    const AP_PIDInfo& get_adrc_info(void) const { return _adrc_info; }

    /// parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /// update_observer_gains - update ESO gains based on observer bandwidth
    void update_observer_gains();

    /// tracking_differentiator - 跟踪微分器 (TD)
    /// @param target: 目标值
    /// @param dt: 采样周期
    void tracking_differentiator(float target, float dt);

    /// extended_state_observer - 扩张观测器 (ESO)
    /// @param measurement: 测量值
    /// @param control_input: 控制输入
    /// @param dt: 采样周期
    void extended_state_observer(float measurement, float control_input, float dt);

    /// nonlinear_state_error_feedback - 非线性状态误差反馈 (NLSEF)
    /// @param dt: 采样周期
    /// @return 控制输出
    float nonlinear_state_error_feedback(float dt);

    // parameters
    AP_Float _wo;      // 观测器带宽 (observer bandwidth)
    AP_Float _kp;      // 比例增益 (proportional gain)
    AP_Float _kd;      // 微分增益 (derivative gain)
    AP_Float _b;       // 补偿系数 (compensation coefficient)

    // ESO gains (calculated from wo)
    float _beta01;     // β01 = 3*wo
    float _beta02;     // β02 = 3*wo^2
    float _beta03;     // β03 = wo^3

    // ESO states (扩张观测器状态)
    float _z1;         // 输出估计值 (estimated output)
    float _z2;         // 输出导数估计值 (estimated derivative)
    float _z3;         // 扰动估计值 (estimated disturbance)

    // TD states (跟踪微分器状态)
    float _v1;         // TD输出1 (TD output 1)
    float _v2;         // TD输出2 (TD output 2)

    // Control components
    float _u0;         // PD控制输出 (PD control output)
    float _disturbance_compensation;  // 扰动补偿量

    // Info structure (for logging)
    AP_PIDInfo _adrc_info;

private:
    const float default_wo;
    const float default_kp;
    const float default_kd;
    const float default_b;
};

