#include "tutorial_mobile_robot/kinematics.hpp"

namespace {
constexpr double TWO_PI = 2.0 * M_PI;
// v = (2πr/60) * rpm  =>  rpm_to_mps = (2πr/60)
inline double RpmToMps(double rpm, double wheel_radius)
{
    return (TWO_PI * wheel_radius / 60.0) * rpm;
}
inline double MpsToRpm(double v, double wheel_radius)
{
    return (wheel_radius > 0.0) ? (v / wheel_radius) * (60.0 / TWO_PI) : 0.0;
}
} // namespace

Kinematics::Kinematics()
: Kinematics(Params{})
{
}

Kinematics::Kinematics(const Params& params)
{
    SetParams(params);
    Reset();
}

void Kinematics::SetParams(const Params& params)
{
    params_ = params;
}

void Kinematics::Reset()
{
    pose_ = Pose{};
}

std::pair<double,double> Kinematics::RobotSpeedToWheelSpeed(double v, double w) const
{
    const double L = params_.wheel_length;
    const double v_left  = v - w * (L * 0.5);
    const double v_right = v + w * (L * 0.5);
    return { v_left, v_right };
}

std::pair<double,double> Kinematics::RPMSpeedToRobotSpeed(int16_t rpm_left, int16_t rpm_right) const
{
    // 기어비 보정: 바퀴 rpm = 모터 rpm / gear
    const double wl = RpmToMps(static_cast<double>(rpm_left)  / params_.gear_ratio, params_.wheel_radius);
    const double wr = RpmToMps(static_cast<double>(rpm_right) / params_.gear_ratio, params_.wheel_radius);

    const double v = 0.5 * (wr + wl);
    const double w = (wr - wl) / params_.wheel_length;
    return { v, w };
}

void Kinematics::UpdateFromVelocity(double linear, double angular, double dt)
{
    if (dt <= 0.0) return;

    // 미드포인트 적분
    const double ds  = linear  * dt;
    const double dth = angular * dt;

    pose_.x     += ds * std::cos(pose_.theta + 0.5 * dth);
    pose_.y     += ds * std::sin(pose_.theta + 0.5 * dth);
    pose_.theta  = NormalizeAngle(pose_.theta + dth);

    pose_.linear_velocity  = linear;
    pose_.angular_velocity = angular;
}

void Kinematics::UpdateFromRPM(int16_t rpm_left, int16_t rpm_right, double dt)
{
    auto vw = RPMSpeedToRobotSpeed(rpm_left, rpm_right);
    UpdateFromVelocity(vw.first, vw.second, dt);
}
