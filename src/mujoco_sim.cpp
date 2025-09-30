#include "tutorial_mobile_robot/mujoco_sim.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace tutorial_mobile_robot {

static inline void YawToQuat(double yaw, double& qw, double& qx, double& qy, double& qz)
{
    const double cz = std::cos(0.5 * yaw);
    const double sz = std::sin(0.5 * yaw);
    qw = cz; qx = 0.0; qy = 0.0; qz = sz; // z-yaw만 고려
}

MjSim::MjSim(const Params& params)
: params_(params)
{
}

MjSim::~MjSim()
{
    if (data_)  { mj_deleteData(data_);  data_  = nullptr; }
    if (model_) { mj_deleteModel(model_); model_ = nullptr; }
}

int MjSim::GetJointQposAddr(const char* joint_name) const
{
    int jid = mj_name2id(model_, mjOBJ_JOINT, joint_name);
    if (jid < 0) return -1;
    return model_->jnt_qposadr[jid];
}

bool MjSim::Load()
{
    char error[1024] = {0};
    model_ = mj_loadXML(params_.mjcf_path.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
        std::cerr << "[MjSim] MJCF load error: " << error << std::endl;
        return false;
    }
    data_ = mj_makeData(model_);
    if (!data_) {
        std::cerr << "[MjSim] mj_makeData failed\n";
        return false;
    }

    qpos_base_        = GetJointQposAddr("base_free");         // 7개(x y z qw qx qy qz)
    qpos_left_wheel_  = GetJointQposAddr("left_wheel_joint");  // 1개
    qpos_right_wheel_ = GetJointQposAddr("right_wheel_joint"); // 1개

    if (qpos_base_ < 0 || qpos_left_wheel_ < 0 || qpos_right_wheel_ < 0) {
        std::cerr << "[MjSim] joint qpos address not found\n";
        return false;
    }

    Reset();
    return true;
}

void MjSim::Reset()
{
    if (!data_) return;

    // 베이스 초기화
    data_->qpos[qpos_base_ + 0] = 0.0; // x
    data_->qpos[qpos_base_ + 1] = 0.0; // y
    data_->qpos[qpos_base_ + 2] = 0.05;// z(고정)
    data_->qpos[qpos_base_ + 3] = 1.0; // qw
    data_->qpos[qpos_base_ + 4] = 0.0; // qx
    data_->qpos[qpos_base_ + 5] = 0.0; // qy
    data_->qpos[qpos_base_ + 6] = 0.0; // qz

    // 바퀴 각도
    data_->qpos[qpos_left_wheel_]  = 0.0;
    data_->qpos[qpos_right_wheel_] = 0.0;

    wheel_w_left_  = 0.0;
    wheel_w_right_ = 0.0;

    mj_forward(model_, data_);
}

void MjSim::SetWheelSpeeds(double left_rad_s, double right_rad_s)
{
    wheel_w_left_  = left_rad_s;
    wheel_w_right_ = right_rad_s;
}

void MjSim::Step(double dt)
{
    if (!data_) return;

    // 1) 바퀴 각도 적분
    data_->qpos[qpos_left_wheel_]  += wheel_w_left_  * dt;
    data_->qpos[qpos_right_wheel_] += wheel_w_right_ * dt;

    // 2) 간단한 평면 키네매틱으로 base 포즈 적분 (동역학 대신)
    // v = r*(wr+wl)/2, w = r*(wr-wl)/L
    const double r = params_.wheel_radius;
    const double L = params_.wheel_length;
    const double v = 0.5 * r * (wheel_w_right_ + wheel_w_left_);
    const double w = (r / L) * (wheel_w_right_ - wheel_w_left_);

    const double x  = data_->qpos[qpos_base_ + 0];
    const double y  = data_->qpos[qpos_base_ + 1];
    double qw = data_->qpos[qpos_base_ + 3];
    double qz = data_->qpos[qpos_base_ + 6];

    // yaw 추출(roll/pitch=0 가정)
    double yaw = std::atan2(2.0 * (qw*qz), 1.0 - 2.0 * (qz*qz));

    double new_x = x + v * std::cos(yaw) * dt;
    double new_y = y + v * std::sin(yaw) * dt;
    double new_yaw = yaw + w * dt;

    double new_qw, new_qx, new_qy, new_qz;
    YawToQuat(new_yaw, new_qw, new_qx, new_qy, new_qz);

    data_->qpos[qpos_base_ + 0] = new_x;
    data_->qpos[qpos_base_ + 1] = new_y;
    data_->qpos[qpos_base_ + 2] = 0.05;
    data_->qpos[qpos_base_ + 3] = new_qw;
    data_->qpos[qpos_base_ + 4] = new_qx;
    data_->qpos[qpos_base_ + 5] = new_qy;
    data_->qpos[qpos_base_ + 6] = new_qz;

    mj_forward(model_, data_);
}

std::pair<double,double> MjSim::GetWheelAngles() const
{
    return { data_->qpos[qpos_left_wheel_], data_->qpos[qpos_right_wheel_] };
}

std::pair<double,double> MjSim::GetWheelSpeeds() const
{
    return { wheel_w_left_, wheel_w_right_ };
}

std::tuple<double,double,double> MjSim::GetBasePose2D() const
{
    const double x = data_->qpos[qpos_base_ + 0];
    const double y = data_->qpos[qpos_base_ + 1];
    const double qw = data_->qpos[qpos_base_ + 3];
    const double qz = data_->qpos[qpos_base_ + 6];
    const double yaw = std::atan2(2.0 * (qw*qz), 1.0 - 2.0 * (qz*qz));
    return {x, y, yaw};
}

std::pair<double,double> MjSim::GetBaseVelocityVW() const
{
    const double r = params_.wheel_radius;
    const double L = params_.wheel_length;
    const double v = 0.5 * r * (wheel_w_right_ + wheel_w_left_);
    const double w = (r / L) * (wheel_w_right_ - wheel_w_left_);
    return {v, w};
}

} // namespace tutorial_mobile_robot
