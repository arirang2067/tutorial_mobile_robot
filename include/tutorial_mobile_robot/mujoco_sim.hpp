#pragma once
#include <string>
#include <utility>
#include <tuple>

// MuJoCo
#include <mujoco/mujoco.h>

namespace tutorial_mobile_robot {

class MjSim {
public:
    struct Params {
        std::string mjcf_path;   // MJCF 파일 경로
        double wheel_radius = 0.08;   // [m]
        double wheel_length = 0.42;   // [m] 바퀴 중심간 거리
    };

    explicit MjSim(const Params& params);
    ~MjSim();

    bool Load();
    void Reset();

    // 외부에서 "휠 각속도[rad/s]"를 직접 넣어줌(컨트롤러/키네매틱스는 ROS 노드가 담당)
    void SetWheelSpeeds(double left_rad_s, double right_rad_s);

    // dt초 동안 한 스텝 적분
    void Step(double dt);

    // 상태 읽기
    std::pair<double,double> GetWheelAngles() const;   // [rad]
    std::pair<double,double> GetWheelSpeeds() const;   // [rad/s]
    std::tuple<double,double,double> GetBasePose2D() const; // x,y,yaw
    std::pair<double,double> GetBaseVelocityVW() const;     // v,w (키네매틱 계산치)

private:
    int GetJointQposAddr(const char* joint_name) const;

private:
    Params params_;

    // MuJoCo 핸들
    mjModel* model_ = nullptr;
    mjData*  data_  = nullptr;

    // qpos index
    int qpos_base_        = -1; // free joint(7개: x y z qw qx qy qz)
    int qpos_left_wheel_  = -1;
    int qpos_right_wheel_ = -1;

    // 입력(외부에서 결정)
    double wheel_w_left_  = 0.0; // [rad/s]
    double wheel_w_right_ = 0.0; // [rad/s]
};

} // namespace tutorial_mobile_robot
