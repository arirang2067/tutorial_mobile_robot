#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "tutorial_mobile_robot/serial_com.hpp"

using namespace std::chrono_literals;

class SerialBridgeNode : public rclcpp::Node
{
public:
    SerialBridgeNode()
    : Node("serial_bridge_node"),
      serial_()
    {
        // 파라미터
        device_   = this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
        baudrate_ = this->declare_parameter<int>("baudrate", 19200);
        left_id_  = this->declare_parameter<int>("left_motor_id", 1);
        right_id_ = this->declare_parameter<int>("right_motor_id", 2);
        pid_set_speed_ = this->declare_parameter<int>("pid_set_speed", 0x10);
        pid_get_speed_ = this->declare_parameter<int>("pid_get_speed", 0x11);
        tx_rate_hz_ = this->declare_parameter<double>("tx_rate_hz", 100.0);

        // 시리얼 설정
        SerialCom::Params params;
        params.device = device_;
        params.baudrate = baudrate_;
        params.pc_id = 0x55;
        params.mdt_id = 0xAA;
        params.mdui_id = 0xAB;
        params.allowed_ids = { (uint8_t)left_id_, (uint8_t)right_id_ };
        serial_.SetParams(params);

        if (!serial_.OpenPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", device_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened", device_.c_str());
        }

        // 퍼블리셔/서브스크립션
        wheel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/wheel_speeds_cmd", 10,
            std::bind(&SerialBridgeNode::HandleWheelCmd, this, std::placeholders::_1));

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 타이머: 시리얼 읽기
        read_timer_ = this->create_wall_timer(
            5ms, std::bind(&SerialBridgeNode::PollSerial, this));
    }

private:
    void HandleWheelCmd(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        // Vector3: x=left ω, y=right ω [rad/s]
        SendSpeed((uint8_t)left_id_, msg->x);
        SendSpeed((uint8_t)right_id_, msg->y);
    }

    void SendSpeed(uint8_t id, double omega)
    {
        // rad/s → uint16_t (임시 변환, 실제 보드 스펙에 맞춰야 함)
        int16_t spd = static_cast<int16_t>(omega * 100.0);
        uint8_t data[2];
        data[0] = (uint8_t)(spd & 0xFF);
        data[1] = (uint8_t)((spd >> 8) & 0xFF);

        serial_.SendPacket(0xAA, 0x55, id, pid_set_speed_, data, 2);
    }

    void PollSerial()
    {
        serial_.ReadAndParse([this](const SerialCom::Frame& f) {
            if (f.pid == pid_get_speed_ && f.length >= 2) {
                int16_t spd_raw = f.data[0] | (f.data[1] << 8);
                double omega = spd_raw / 100.0; // rad/s (역변환)

                sensor_msgs::msg::JointState js;
                js.header.stamp = this->now();
                js.name = { (f.id == left_id_) ? "left_wheel_joint" : "right_wheel_joint" };
                js.velocity = { omega };
                joint_pub_->publish(js);
            }
        });
    }

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wheel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr read_timer_;

    // Serial
    SerialCom serial_;

    // 파라미터
    std::string device_;
    int baudrate_;
    int left_id_, right_id_;
    int pid_set_speed_, pid_get_speed_;
    double tx_rate_hz_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
