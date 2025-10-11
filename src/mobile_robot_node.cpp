#include <memory>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "tutorial_mobile_robot/controller.hpp"
#include "tutorial_mobile_robot/kinematics.hpp"

// 함수: PascalCase, 변수: snake_case

namespace {
// 필수 파라미터 강제 헬퍼: 없으면 즉시 예외
template<typename T>
T GetRequiredParam(const rclcpp::Node& node, const std::string& name)
{
    T value{};
    if (!node.get_parameter(name, value)) {
        throw std::runtime_error(
            "Required parameter '" + name + "' is not set. "
            "Set it via launch/YAML (parameters=[...]).");
    }
    return value;
}
} // namespace

class TutorialMotorNode : public rclcpp::Node
{
public:
    explicit TutorialMotorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("mobile_robot_node", options),
      loop_hz_(100.0),
      publish_tf_(true),
      publish_local_outputs_(true),
      controller_(Controller::Params{}),
      kinematics_(Kinematics::Params{}),
      left_wheel_pos_rad_(0.0),
      right_wheel_pos_rad_(0.0),
      last_time_(this->get_clock()->now())
    {
        LoadParamsOnlyFromOverrides();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        if (publish_local_outputs_) {
            odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);
            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
            if (publish_tf_) {
                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            }
        }

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", qos,
            std::bind(&TutorialMotorNode::HandleCmdVel, this, std::placeholders::_1));

        wheel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/wheel_speeds_cmd", qos);

        const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TutorialMotorNode::RunTimer, this));
    }

private:
    // ---------- 파라미터(런치/YAML에서만) ----------
    void LoadParamsOnlyFromOverrides()
    {
        // Frames / loop / outputs
        base_frame_id_        = GetRequiredParam<std::string>(*this, "base_frame_id");
        odom_frame_id_        = GetRequiredParam<std::string>(*this, "odom_frame_id");
        left_joint_name_      = GetRequiredParam<std::string>(*this, "left_joint_name");
        right_joint_name_     = GetRequiredParam<std::string>(*this, "right_joint_name");
        loop_hz_              = GetRequiredParam<double>(*this, "loop_hz");
        publish_tf_           = GetRequiredParam<bool>(*this, "publish_tf");
        publish_local_outputs_= GetRequiredParam<bool>(*this, "publish_local_outputs");

        // Kinematics
        Kinematics::Params kp;
        kp.wheel_radius = GetRequiredParam<double>(*this, "wheel_radius");
        kp.wheel_length = GetRequiredParam<double>(*this, "wheel_length");
        kp.gear_ratio   = GetRequiredParam<double>(*this, "gear_ratio");
        kp.max_rpm      = GetRequiredParam<int>(*this, "max_rpm");
        kinematics_.SetParams(kp);

        // Controller
        Controller::Params cp;
        cp.use_rate_limit        = GetRequiredParam<bool>(*this, "use_rate_limit");
        cp.bound_cmd_speed       = GetRequiredParam<double>(*this, "bound_cmd_speed");
        cp.add_cmd_speed         = GetRequiredParam<double>(*this, "add_cmd_speed");
        cp.bound_cmd_ang_speed   = GetRequiredParam<double>(*this, "bound_cmd_ang_speed");
        cp.add_cmd_ang_speed     = GetRequiredParam<double>(*this, "add_cmd_ang_speed");
        cp.max_linear_accel      = GetRequiredParam<double>(*this, "max_linear_accel");
        cp.max_angular_accel     = GetRequiredParam<double>(*this, "max_angular_accel");
        cp.deadzone_linear       = GetRequiredParam<double>(*this, "deadzone_linear");
        cp.deadzone_angular      = GetRequiredParam<double>(*this, "deadzone_angular");
        cp.lowpass_alpha_linear  = GetRequiredParam<double>(*this, "lowpass_alpha_linear");
        cp.lowpass_alpha_angular = GetRequiredParam<double>(*this, "lowpass_alpha_angular");
        cp.limit_linear_abs      = GetRequiredParam<double>(*this, "limit_linear_abs");
        cp.limit_angular_abs     = GetRequiredParam<double>(*this, "limit_angular_abs");
        controller_.SetParams(cp);

        RCLCPP_INFO(this->get_logger(),
            "Params loaded from overrides only. base=%s odom=%s loop=%.1f publish_tf=%s",
            base_frame_id_.c_str(), odom_frame_id_.c_str(), loop_hz_,
            publish_tf_ ? "true" : "false");
    }

    // ---------- 콜백/루프 ----------
    void HandleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        controller_.SetReference(msg->linear.x, msg->angular.z);
    }

    void RunTimer()
    {
        const auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0) dt = 1.0 / loop_hz_;
        last_time_ = now;

        controller_.Update(dt);
        const auto cmd = controller_.GetCommand(); // v,w

        // 1) 시뮬 구동용: 휠 각속도 명령 퍼블리시
        {
            const auto wheel_v = kinematics_.RobotSpeedToWheelSpeed(cmd.linear, cmd.angular); // [m/s]
            const double r = kinematics_.GetParams().wheel_radius;
            geometry_msgs::msg::Vector3 v3;
            v3.x = (r > 0.0) ? (wheel_v.first  / r) : 0.0; // left ω [rad/s]
            v3.y = (r > 0.0) ? (wheel_v.second / r) : 0.0; // right ω [rad/s]
            v3.z = 0.0;
            wheel_cmd_pub_->publish(v3);
        }

        // 2) (옵션) 자체 퍼블리시 — 필요 시 주석 해제
        // if (publish_local_outputs_) {
        //     kinematics_.UpdateFromVelocity(cmd.linear, cmd.angular, dt);
        //     const auto pose = kinematics_.GetPose();
        //     PublishJointStates(cmd, dt);
        //     PublishOdomAndTf(pose, now);
        // }
    }

    // ---------- 퍼블리시 유틸 ----------
    void PublishJointStates(const Controller::Command& cmd, double dt)
    {
        const auto wheel_v = kinematics_.RobotSpeedToWheelSpeed(cmd.linear, cmd.angular);
        const double r = kinematics_.GetParams().wheel_radius;
        const double left_w  = (r > 0.0) ? wheel_v.first  / r : 0.0;
        const double right_w = (r > 0.0) ? wheel_v.second / r : 0.0;

        left_wheel_pos_rad_  += left_w  * dt;
        right_wheel_pos_rad_ += right_w * dt;

        sensor_msgs::msg::JointState js;
        js.header.stamp = this->get_clock()->now();
        js.name = { left_joint_name_, right_joint_name_ };
        js.position = { left_wheel_pos_rad_, right_wheel_pos_rad_ };
        js.velocity = { left_w, right_w };
        joint_pub_->publish(js);
    }

    void PublishOdomAndTf(const Kinematics::Pose& pose, const rclcpp::Time& stamp)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pose.theta);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id  = base_frame_id_;

        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
        odom.pose.covariance[0]  = 0.05;
        odom.pose.covariance[7]  = 0.05;
        odom.pose.covariance[35] = 0.10;

        odom.twist.twist.linear.x  = pose.linear_velocity;
        odom.twist.twist.linear.y  = 0.0;
        odom.twist.twist.angular.z = pose.angular_velocity;
        for (int i = 0; i < 36; ++i) odom.twist.covariance[i] = 0.0;
        odom.twist.covariance[0]  = 0.05;
        odom.twist.covariance[35] = 0.10;

        odom_pub_->publish(odom);

        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = odom_frame_id_;
            tf_msg.child_frame_id  = base_frame_id_;
            tf_msg.transform.translation.x = pose.x;
            tf_msg.transform.translation.y = pose.y;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_msg.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

private:
    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr wheel_cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 파라미터
    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::string left_joint_name_;
    std::string right_joint_name_;
    double loop_hz_;
    bool publish_tf_;
    bool publish_local_outputs_;

    // 상태
    Controller controller_;
    Kinematics kinematics_;
    double left_wheel_pos_rad_;
    double right_wheel_pos_rad_;
    rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // 런치/YAML 오버라이드를 자동 선언 → 코드에서는 get_parameter만 사용
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<TutorialMotorNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
