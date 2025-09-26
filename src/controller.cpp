#include "tutorial_ros2_motor/controller.hpp"

Controller::Controller()
: Controller(Params{})
{
}

Controller::Controller(const Params& params)
{
    SetParams(params);
    Reset();
}

void Controller::SetParams(const Params& params)
{
    params_ = params;
    // 리미트 sanity
    if (params_.limit_linear_abs < 0.0)  params_.limit_linear_abs  = 0.0;
    if (params_.limit_angular_abs < 0.0) params_.limit_angular_abs = 0.0;
}

void Controller::SetReference(double linear_ref, double angular_ref)
{
    ref_linear_  = Clamp(linear_ref,  -params_.limit_linear_abs,  params_.limit_linear_abs);
    ref_angular_ = Clamp(angular_ref, -params_.limit_angular_abs, params_.limit_angular_abs);
}

void Controller::Reset()
{
    ref_linear_ = ref_angular_ = 0.0;
    filt_linear_ = filt_angular_ = 0.0;
    cmd_linear_ = cmd_angular_ = 0.0;
}

void Controller::Update(double dt)
{
    if (dt <= 0.0) return;

    // 1) 데드존
    double ref_v = ApplyDeadzone(ref_linear_,  params_.deadzone_linear);
    double ref_w = ApplyDeadzone(ref_angular_, params_.deadzone_angular);

    // 2) 1차 저역통과 (y += a*(u-y))
    if (params_.lowpass_alpha_linear > 0.0)
        filt_linear_ += params_.lowpass_alpha_linear * (ref_v - filt_linear_);
    else
        filt_linear_  = ref_v;

    if (params_.lowpass_alpha_angular > 0.0)
        filt_angular_ += params_.lowpass_alpha_angular * (ref_w - filt_angular_);
    else
        filt_angular_  = ref_w;

    // 3) 스무딩 모드
    if (params_.use_rate_limit)
    {
        RateLimitSmoothing(dt, filt_linear_,  cmd_linear_,  params_.max_linear_accel);
        RateLimitSmoothing(dt, filt_angular_, cmd_angular_, params_.max_angular_accel);
    }
    else
    {
        StepSmoothing(dt, filt_linear_,  cmd_linear_,  params_.bound_cmd_speed,     params_.add_cmd_speed);
        StepSmoothing(dt, filt_angular_, cmd_angular_, params_.bound_cmd_ang_speed, params_.add_cmd_ang_speed);
    }

    // 4) 안전 리미트
    cmd_linear_  = Clamp(cmd_linear_,  -params_.limit_linear_abs,  params_.limit_linear_abs);
    cmd_angular_ = Clamp(cmd_angular_, -params_.limit_angular_abs, params_.limit_angular_abs);
}

void Controller::StepSmoothing(double /*dt*/, double ref, double& cmd, double bound, double add)
{
    if (cmd + bound < ref)
        cmd += add;
    else if (cmd - bound > ref)
        cmd -= add;
    else
        cmd = 0.8 * cmd + 0.2 * ref; // 작은 오차는 완만히 수렴
}

void Controller::RateLimitSmoothing(double dt, double ref, double& cmd, double max_accel)
{
    const double delta = ref - cmd;
    const double max_step = max_accel * dt;
    const double step = Clamp(delta, -max_step, max_step);
    cmd += step;
}
