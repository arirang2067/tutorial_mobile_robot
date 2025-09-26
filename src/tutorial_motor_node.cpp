#include <memory>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tutorial_ros2_motor/controller.hpp"
#include "tutorial_ros2_motor/kinematics.hpp"
#include "tutorial_ros2_motor/serial_com.hpp"

// ===== 통신 프로토콜 상수 =====
static constexpr uint8_t PID_PNT_VEL_CMD      = 207; // 속도 명령
static constexpr uint8_t PID_PNT_MAIN_DATA    = 210; // 메인 상태(피드백)
static constexpr uint8_t REQUEST_PNT_MAIN_DATA= 2;   // req_monitor_id 값
static constexpr uint8_t ID_ALL               = 0xFE;

// ===== 수신 프레임 해석용 구조체 (PID 210) =====
// 원래 packed struct와 동일한 필드 순서/크기로 맞춥니다.
#pragma pack(push, 1)
struct PidPntMainData
{
    int16_t rpm_id1;
    int16_t current_id1;
    uint8_t mtr_state_id1;  // MOTOR_STATE_t.val
    int32_t mtr_pos_id1;

    int16_t rpm_id2;
    int16_t current_id2;
    uint8_t mtr_state_id2;  // MOTOR_STATE_t.val
    int32_t mtr_pos_id2;
};
#pragma pack(pop)

// 함수: PascalCase, 변수: snake_case
class TutorialMotorNode : public rclcpp::Node
{
public:
    TutorialMotorNode()
    : Node("tutorial_ros2_motor_node")
    {
        // ---- 멤버 기본값 ----
        loop_hz_            = 100.0;
        publish_tf_         = true;
        use_hardware_       = false;
        serial_device_      = "/dev/ttywhl";
        serial_baudrate_    = 19200;
        pc_id_              = 172; // MID_PC
        mdt_id_             = 183; // MID_MDT
        mdui_id_            = 184; // MID_MDUI
        use_mdui_           = false;
        motor_bus_id_       = 1;   // 내부 모터 ID (또는 0xFE 브로드캐스트)
        left_wheel_pos_rad_ = 0.0;
        right_wheel_pos_rad_= 0.0;
        have_measured_      = false;
        meas_rpm_left_      = 0.0;
        meas_rpm_right_     = 0.0;
        last_time_          = this->get_clock()->now();

        DeclareAndLoadParams(); // 파라미터 로드(kinematics/controller/serial)

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);

        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", qos,
            std::bind(&TutorialMotorNode::HandleCmdVel, this, std::placeholders::_1));

        // 하드웨어 모드면 직렬 포트 오픈
        if (use_hardware_) {
            SerialCom::Params sp;
            sp.device   = serial_device_;
            sp.baudrate = static_cast<unsigned int>(serial_baudrate_);
            sp.pc_id    = static_cast<uint8_t>(pc_id_);
            sp.mdt_id   = static_cast<uint8_t>(mdt_id_);
            sp.mdui_id  = static_cast<uint8_t>(mdui_id_);
            // 허용 모터 ID (비우면 {1, 0xFE} 자동 허용)
            sp.allowed_ids = { static_cast<uint8_t>(motor_bus_id_), ID_ALL };

            com_.SetParams(sp);
            if (!com_.OpenPort()) {
                RCLCPP_ERROR(this->get_logger(), "Serial open failed: %s", serial_device_.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Serial opened: %s @ %d", serial_device_.c_str(), serial_baudrate_);
            }
        }

        const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TutorialMotorNode::RunTimer, this));
    }

private:
    // ---------- 파라미터 ----------
    void DeclareAndLoadParams()
    {
        base_frame_id_     = this->declare_parameter<std::string>("base_frame_id", "base_link");
        odom_frame_id_     = this->declare_parameter<std::string>("odom_frame_id", "odom");
        left_joint_name_   = this->declare_parameter<std::string>("left_joint_name",  "wheel_left_joint");
        right_joint_name_  = this->declare_parameter<std::string>("right_joint_name", "wheel_right_joint");
        loop_hz_           = this->declare_parameter<double>("loop_hz", 100.0);
        publish_tf_        = this->declare_parameter<bool>("publish_tf", true);

        // Kinematics
        Kinematics::Params kp;
        kp.wheel_radius = this->declare_parameter<double>("wheel_radius", 0.08);
        kp.wheel_length = this->declare_parameter<double>("wheel_length", 0.42);
        kp.gear_ratio   = this->declare_parameter<double>("gear_ratio", 4.0);
        kp.max_rpm      = this->declare_parameter<int>("max_rpm", 350);
        kinematics_.SetParams(kp);

        // Controller
        Controller::Params cp;
        cp.use_rate_limit        = this->declare_parameter<bool>("use_rate_limit", false);
        cp.bound_cmd_speed       = this->declare_parameter<double>("bound_cmd_speed", 0.15);
        cp.add_cmd_speed         = this->declare_parameter<double>("add_cmd_speed", 0.03);
        cp.bound_cmd_ang_speed   = this->declare_parameter<double>("bound_cmd_ang_speed", 0.30);
        cp.add_cmd_ang_speed     = this->declare_parameter<double>("add_cmd_ang_speed", 0.06);
        cp.max_linear_accel      = this->declare_parameter<double>("max_linear_accel", 0.6);
        cp.max_angular_accel     = this->declare_parameter<double>("max_angular_accel", 1.5);
        cp.deadzone_linear       = this->declare_parameter<double>("deadzone_linear", 0.03);
        cp.deadzone_angular      = this->declare_parameter<double>("deadzone_angular", 0.03);
        cp.lowpass_alpha_linear  = this->declare_parameter<double>("lowpass_alpha_linear", 0.0);
        cp.lowpass_alpha_angular = this->declare_parameter<double>("lowpass_alpha_angular", 0.0);
        cp.limit_linear_abs      = this->declare_parameter<double>("limit_linear_abs", 1.5);
        cp.limit_angular_abs     = this->declare_parameter<double>("limit_angular_abs", 3.0);
        controller_.SetParams(cp);

        // Serial / Hardware
        use_hardware_    = this->declare_parameter<bool>("use_hardware", false);
        serial_device_   = this->declare_parameter<std::string>("serial_device", "/dev/ttywhl");
        serial_baudrate_ = this->declare_parameter<int>("serial_baudrate", 19200);

        pc_id_           = this->declare_parameter<int>("pc_id", 172);
        mdt_id_          = this->declare_parameter<int>("mdt_id", 183);
        mdui_id_         = this->declare_parameter<int>("mdui_id", 184);
        use_mdui_        = this->declare_parameter<bool>("use_mdui", false);
        motor_bus_id_    = this->declare_parameter<int>("motor_bus_id", 1);
    }

    // ---------- 콜백 ----------
    void HandleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        controller_.SetReference(msg->linear.x, msg->angular.z);
    }

    // ---------- 주기 루프 ----------
    void RunTimer()
    {
        const auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0) dt = 1.0 / loop_hz_;
        last_time_ = now;

        controller_.Update(dt);
        const auto cmd = controller_.GetCommand();

        // 하드웨어 송신(옵션)
        if (use_hardware_ && com_.IsOpen()) {
            auto rpm_pair = ComputeWheelRPMs(cmd.linear, cmd.angular);
            SendVelToDriver(rpm_pair.first, rpm_pair.second);

            // 수신 파싱(옵션): PID 210 들어오면 측정 RPM 업데이트
            PollSerial();
        }

        // 오돔/조인트는 "실측 RPM"이 있으면 그것을 우선 사용, 없으면 명령값 기반 모델 사용
        if (have_measured_) {
            PublishJointStatesFromMeasured(dt);
            // 측정 기반 선속/각속 추정
            const double r = kinematics_.GetParams().wheel_radius;
            const double L = kinematics_.GetParams().wheel_length;
            const double wl = (meas_rpm_left_  * 2.0 * M_PI) / 60.0; // [rad/s] (wheel rpm -> rad/s)
            const double wr = (meas_rpm_right_ * 2.0 * M_PI) / 60.0;

            const double v = 0.5 * r * (wr + wl);
            const double w = (r / L) * (wr - wl);
            kinematics_.UpdateFromVelocity(v, w, dt);
        } else {
            // 시뮬레이션(명령 기반) 업데이트
            kinematics_.UpdateFromVelocity(cmd.linear, cmd.angular, dt);
            PublishJointStatesFromCommand(cmd, dt);
        }

        const auto pose = kinematics_.GetPose();
        PublishOdomAndTf(pose, now);
    }

    // ---------- 직렬 보조 ----------
    std::pair<int16_t,int16_t> ComputeWheelRPMs(double linear, double angular)
    {
        // 바퀴 선속도
        auto wheel_v = kinematics_.RobotSpeedToWheelSpeed(linear, angular);
        const double r = kinematics_.GetParams().wheel_radius;
        const double red = kinematics_.GetParams().gear_ratio;
        const int max_rpm = kinematics_.GetParams().max_rpm;

        // v = r * ω  → wheel_rpm = (v / (2πr)) * 60
        // 모터 rpm = wheel_rpm * reduction
        auto ToMotorRPM = [&](double v)->int16_t {
            double wheel_rpm = (r > 0.0) ? (v / (2.0 * M_PI * r)) * 60.0 : 0.0;
            double motor_rpm = wheel_rpm * red;
            motor_rpm = std::clamp(motor_rpm, -static_cast<double>(max_rpm), static_cast<double>(max_rpm));
            return static_cast<int16_t>(std::lround(motor_rpm));
        };

        int16_t rpm_left  = ToMotorRPM(wheel_v.first);
        int16_t rpm_right = ToMotorRPM(wheel_v.second);
        return {rpm_left, rpm_right};
    }

    static inline void WriteInt16LE(uint8_t* p, int16_t v)
    {
        p[0] = static_cast<uint8_t>(v & 0xFF);
        p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    }

    void SendVelToDriver(int16_t rpm_left, int16_t rpm_right)
    {
        // PID_PNT_VEL_CMD payload(7 bytes):
        // [0]=enable_id1(1), [1..2]=rpm_id1, [3]=enable_id2(2), [4..5]=rpm_id2, [6]=req_monitor_id
        uint8_t payload[7];
        payload[0] = 1; // left enable
        WriteInt16LE(&payload[1], rpm_left);
        payload[3] = 2; // right enable
        WriteInt16LE(&payload[4], rpm_right);
        payload[6] = 0; // or REQUEST_PNT_MAIN_DATA(2)로 바꾸면 주기적으로 210 응답을 유도할 수 있음

        const uint8_t rmid = static_cast<uint8_t>(use_mdui_ ? mdui_id_ : mdt_id_);
        const uint8_t pcid = static_cast<uint8_t>(pc_id_);
        const uint8_t id   = static_cast<uint8_t>(motor_bus_id_);

        (void)com_.SendPacket(rmid, pcid, id, PID_PNT_VEL_CMD, payload, sizeof(payload));
    }

    void PollSerial()
    {
        com_.ReadAndParse([&](const SerialCom::Frame& f){
            if (f.pid == PID_PNT_MAIN_DATA && f.length >= sizeof(PidPntMainData)) {
                PidPntMainData data{};
                std::memcpy(&data, f.data.data(), sizeof(PidPntMainData));

                // 원 코드와 동일: 감속비 반영(반올림) 후 바퀴 rpm으로 보정
                auto NormalizeRpm = [&](int16_t raw)->double {
                    const int red = kinematics_.GetParams().gear_ratio;
                    if (raw >= 0) return static_cast<double>((raw + red/2) / red);
                    else          return static_cast<double>((raw - red/2) / red);
                };

                meas_rpm_left_  = NormalizeRpm(data.rpm_id1);
                meas_rpm_right_ = NormalizeRpm(data.rpm_id2);
                have_measured_  = true;
            }
        });
    }

    // ---------- 퍼블리시 ----------
    void PublishJointStatesFromCommand(const Controller::Command& cmd, double dt)
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

    void PublishJointStatesFromMeasured(double dt)
    {
        // meas_rpm_* 는 "바퀴 rpm"으로 보정된 값
        const double wl = (meas_rpm_left_  * 2.0 * M_PI) / 60.0;
        const double wr = (meas_rpm_right_ * 2.0 * M_PI) / 60.0;

        left_wheel_pos_rad_  += wl * dt;
        right_wheel_pos_rad_ += wr * dt;

        sensor_msgs::msg::JointState js;
        js.header.stamp = this->get_clock()->now();
        js.name = { left_joint_name_, right_joint_name_ };
        js.position = { left_wheel_pos_rad_, right_wheel_pos_rad_ };
        js.velocity = { wl, wr };
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
        odom.pose.covariance[0]  = 0.05; // x
        odom.pose.covariance[7]  = 0.05; // y
        odom.pose.covariance[35] = 0.10; // yaw

        odom.twist.twist.linear.x  = pose.linear_velocity;
        odom.twist.twist.linear.y  = 0.0;
        odom.twist.twist.angular.z = pose.angular_velocity;

        for (int i = 0; i < 36; ++i) odom.twist.covariance[i] = 0.0;
        odom.twist.covariance[0]  = 0.05; // v
        odom.twist.covariance[35] = 0.10; // w

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
    // ROS I/F
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
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

    // 하드웨어/직렬 파라미터
    bool use_hardware_;
    std::string serial_device_;
    int serial_baudrate_;
    int pc_id_;
    int mdt_id_;
    int mdui_id_;
    bool use_mdui_;
    int motor_bus_id_;

    // 상태
    Controller controller_;
    Kinematics kinematics_;
    double left_wheel_pos_rad_;
    double right_wheel_pos_rad_;
    rclcpp::Time last_time_;

    // 직렬 통신
    SerialCom com_;
    bool   have_measured_;
    double meas_rpm_left_;   // 바퀴 rpm(보정 후)
    double meas_rpm_right_;  // 바퀴 rpm(보정 후)
};

// main
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TutorialMotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
