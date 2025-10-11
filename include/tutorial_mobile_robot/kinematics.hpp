#ifndef TUTORIAL_MOBILE_ROBOT_KINEMATICS_HPP_
#define TUTORIAL_MOBILE_ROBOT_KINEMATICS_HPP_

#include <utility>
#include <cmath>
#include <cstdint>

struct Kinematics
{
    struct Params
    {
        double wheel_radius = 0.065;   // Ø130mm [m]
        double wheel_length = 0.4465;  // 바퀴 중심간 거리 [m]
        double gear_ratio   = 1.0;  // 감속비
        int    max_rpm      = 400;  // 모터 최대 rpm
    };

    struct Pose
    {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double linear_velocity  = 0.0;
        double angular_velocity = 0.0;
    };

    Kinematics();
    explicit Kinematics(const Params& params);

    void SetParams(const Params& params);
    const Params& GetParams() const { return params_; }

    // v,w 기반 적분
    void UpdateFromVelocity(double linear, double angular, double dt);

    // 좌/우 RPM 기반 업데이트
    void UpdateFromRPM(int16_t rpm_left, int16_t rpm_right, double dt);

    // 변환 유틸
    std::pair<double,double> RobotSpeedToWheelSpeed(double v, double w) const;
    std::pair<double,double> RPMSpeedToRobotSpeed(int16_t rpm_left, int16_t rpm_right) const;

    const Pose& GetPose() const { return pose_; }
    void Reset();

private:
    static inline double NormalizeAngle(double a)
    {
        while (a >  M_PI) a -= 2.0*M_PI;
        while (a < -M_PI) a += 2.0*M_PI;
        return a;
    }

private:
    Params params_;
    Pose   pose_;
};

#endif // TUTORIAL_MOBILE_ROBOT_KINEMATICS_HPP_
