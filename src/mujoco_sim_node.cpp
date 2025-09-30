#include <memory>
#include <string>
#include <chrono>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tutorial_mobile_robot/mujoco_sim.hpp"

using tutorial_mobile_robot::MjSim;

class MujocoSimNode : public rclcpp::Node
{
public:
    MujocoSimNode()
    : Node("mujoco_diff_drive_sim")
    {
        // 파라미터
        base_frame_id_    = this->declare_parameter<std::string>("base_frame_id", "base_link");
        odom_frame_id_    = this->declare_parameter<std::string>("odom_frame_id", "odom");
        left_joint_name_  = this->declare_parameter<std::string>("left_joint_name",  "wheel_left_joint");
        right_joint_name_ = this->declare_parameter<std::string>("right_joint_name", "wheel_right_joint");
        loop_hz_          = this->declare_parameter<double>("loop_hz", 200.0);
        publish_tf_       = this->declare_parameter<bool>("publish_tf", true);

        // MJCF 경로
        std::string mjcf_path = this->declare_parameter<std::string>("mjcf_path", "");
        if (mjcf_path.empty()) {
            auto share = ament_index_cpp::get_package_share_directory("tutorial_mobile_robot");
            mjcf_path = share + "/assets/mobile_robot.xml";
        }

        // MuJoCo 초기화
        MjSim::Params sp;
        sp.mjcf_path    = mjcf_path;
        sp.wheel_radius = this->declare_parameter<double>("wheel_radius", 0.065);
        sp.wheel_length = this->declare_parameter<double>("wheel_length", 0.4465);

        sim_ = std::make_unique<MjSim>(sp);
        if (!sim_->Load()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load MJCF: %s", mjcf_path.c_str());
            throw std::runtime_error("MJCF load failed");
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }

        // ▲ 여기! 외부(컨트롤러/키네매틱스)에서 온 휠 각속도 명령 구독
        wheel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/wheel_speeds_cmd", qos,
            std::bind(&MujocoSimNode::HandleWheelCmd, this, std::placeholders::_1));

        last_time_ = this->get_clock()->now();

        const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&MujocoSimNode::RunTimer, this));

        RCLCPP_INFO(this->get_logger(), "MuJoCo sim ready. MJCF: %s", mjcf_path.c_str());
    }

private:
    void HandleWheelCmd(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        // x=left_rad_s, y=right_rad_s
        sim_->SetWheelSpeeds(msg->x, msg->y);
    }

    void RunTimer()
    {
        const auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0) dt = 1.0 / loop_hz_;
        last_time_ = now;

        sim_->Step(dt);

        // JointStates
        const auto angles = sim_->GetWheelAngles();
        const auto speeds = sim_->GetWheelSpeeds();

        sensor_msgs::msg::JointState js;
        js.header.stamp = now;
        js.name = {left_joint_name_, right_joint_name_};
        js.position = { angles.first, angles.second };
        js.velocity = { speeds.first, speeds.second };
        joint_pub_->publish(js);

        // Odom + TF
        const auto [x,y,yaw] = sim_->GetBasePose2D();
        const auto [v,w]     = sim_->GetBaseVelocityVW();

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id  = base_frame_id_;

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x  = v;
        odom.twist.twist.angular.z = w;
        odom_pub_->publish(odom);

        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = odom_frame_id_;
            tf_msg.child_frame_id  = base_frame_id_;
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
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
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wheel_cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 파라미터/상태
    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::string left_joint_name_;
    std::string right_joint_name_;
    double loop_hz_;
    bool publish_tf_;
    rclcpp::Time last_time_;

    // MuJoCo
    std::unique_ptr<MjSim> sim_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MujocoSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
