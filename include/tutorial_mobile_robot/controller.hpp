#ifndef TUTORIAL_MOBILE_ROBOT_CONTROLLER_HPP_
#define TUTORIAL_MOBILE_ROBOT_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

struct Controller
{
    struct Params
    {
        // step 스무딩 파라미터
        double bound_cmd_speed       = 0.15; // [m/s] 목표와 현재의 허용 오차(선속도)
        double add_cmd_speed         = 0.03; // [m/s] tick당 선속도 증분
        double bound_cmd_ang_speed   = 0.30; // [rad/s]
        double add_cmd_ang_speed     = 0.06; // [rad/s]

        // rate limit(가속도 제한) 파라미터
        double max_linear_accel      = 0.6;  // [m/s^2]
        double max_angular_accel     = 1.5;  // [rad/s^2]

        // 모드/필터/리미트
        bool   use_rate_limit        = false; // false: step, true: accel limit
        double deadzone_linear       = 0.03;  // |v|<deadzone -> 0
        double deadzone_angular      = 0.03;  // |w|<deadzone -> 0
        double lowpass_alpha_linear  = 0.0;   // 0~1 (0=off), y+=a(u-y)
        double lowpass_alpha_angular = 0.0;
        double limit_linear_abs      = 1.5;   // [m/s]
        double limit_angular_abs     = 3.0;   // [rad/s]
    };

    struct Command
    {
        double linear  = 0.0; // [m/s]
        double angular = 0.0; // [rad/s]
    };

    Controller();
    explicit Controller(const Params& params);

    void SetParams(const Params& params);
    void SetReference(double linear_ref, double angular_ref); // 목표 명령 저장
    void Update(double dt);                                   // dt초 동안 스무딩
    Command GetCommand() const { return { cmd_linear_, cmd_angular_ }; }
    void Reset();

private:
    static inline double Clamp(double x, double lo, double hi)
    {
        return std::max(lo, std::min(x, hi));
    }
    static inline double ApplyDeadzone(double x, double dz)
    {
        return (std::fabs(x) < dz) ? 0.0 : x;
    }
    void StepSmoothing(double dt, double ref, double& cmd, double bound, double add);
    void RateLimitSmoothing(double dt, double ref, double& cmd, double max_accel);

private:
    Params params_;

    // 입력 참조 (사용자/상위 노드가 준 값)
    double ref_linear_  = 0.0; // [m/s]
    double ref_angular_ = 0.0; // [rad/s]

    // 내부 필터 출력
    double filt_linear_  = 0.0;
    double filt_angular_ = 0.0;

    // 최종 명령(스무딩 결과)
    double cmd_linear_  = 0.0;
    double cmd_angular_ = 0.0;
};

#endif // TUTORIAL_MOBILE_ROBOT_CONTROLLER_HPP_
