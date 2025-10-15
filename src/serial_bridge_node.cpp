#include <memory>
#include <string>
#include <cmath>
#include <cstdint>
#include <vector>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "tutorial_mobile_robot/serial_com.hpp"

// ── 프로토콜 상수
static constexpr uint8_t ID_ALL  = 0xFE;
static constexpr uint8_t MID_MDUI = 184;
static constexpr uint8_t MID_MDT  = 183;
static constexpr uint8_t MID_PC   = 172;

enum : uint8_t {
  PID_REQ_PID_DATA  = 4,
  PID_COMMAND       = 10,
  PID_PNT_VEL_CMD   = 207,
  PID_PNT_MAIN_DATA = 210,
};

enum : uint8_t {
  CMD_PNT_MAIN_DATA_BC_ON = 61,
};

static constexpr uint8_t REQUEST_PNT_MAIN_DATA = 2;

// ── 페이로드(전송)
#pragma pack(push,1)
struct PntVelCmdPayload {
  uint8_t  enable_id1;
  int16_t  rpm_id1;
  uint8_t  enable_id2;
  int16_t  rpm_id2;
  uint8_t  req_monitor_id;
};
#pragma pack(pop)

// ── 피드백(수신)
#pragma pack(push,1)
struct MotorStateBits {
  uint8_t Alarm:1, CtrlFail:1, OverVolt:1, OverTemp:1,
          OverLoad:1, HallFail:1, InvVel:1, Stall:1;
};
union MotorState { uint8_t val; MotorStateBits bits; };

struct PntMainData {
  int16_t     rpm_id1;
  int16_t     current_id1;
  MotorState  mtr_state_id1;
  int32_t     mtr_pos_id1;
  int16_t     rpm_id2;
  int16_t     current_id2;
  MotorState  mtr_state_id2;
  int32_t     mtr_pos_id2;
};
#pragma pack(pop)

// ─────────────────────────────────────────────────────────────────────────────
// 파라미터 강제 헬퍼: 없으면 즉시 예외
namespace {
template<typename T>
T GetParam(const rclcpp::Node& node, const std::string& name) {
  T v{};
  if (!node.get_parameter(name, v)) {
    throw std::runtime_error("Required parameter '" + name + "' is not set. Supply via YAML/launch.");
  }
  return v;
}

// 정수로 받고 범위 확인 후 작은 타입으로 캐스팅 (예: uint8_t)
template<typename SmallIntT>
SmallIntT GetParamUint8(const rclcpp::Node& node, const std::string& name) {
  static_assert(std::is_same<SmallIntT, uint8_t>::value, "SmallIntT must be uint8_t");
  int tmp{};
  if (!node.get_parameter(name, tmp)) {
    throw std::runtime_error("Required parameter '" + name + "' is not set. Supply via YAML/launch.");
  }
  if (tmp < 0 || tmp > 255) {
    throw std::runtime_error("Parameter '" + name + "' must be in [0,255], got " + std::to_string(tmp));
  }
  return static_cast<uint8_t>(tmp);
}
} // namespace

class SerialBridgeNode : public rclcpp::Node
{
public:
  SerialBridgeNode()
  : Node(
      "serial_bridge_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    )
  {
    LoadParams();      // ← declare 없이 필수 파라미터 강제 로드
    ConfigureSerial();
    SetupRos();

    last_time_    = this->get_clock()->now();
    last_rx_time_ = this->get_clock()->now();
    last_cmd_time_= this->get_clock()->now();
  }

private:
  // ========= 파라미터 =========
  void LoadParams()
  {
    // 포트/프레임
    device_   = GetParam<std::string>(*this, "device");
    baudrate_ = GetParam<int>(*this, "baudrate");
    rmid_     = GetParamUint8<uint8_t>(*this, "rmid");
    pcid_     = GetParamUint8<uint8_t>(*this, "pcid");
    fixed_id_ = GetParamUint8<uint8_t>(*this, "fixed_id");

    // 변환/극성
    invert_left_   = GetParam<bool>(*this, "invert_left");
    invert_right_  = GetParam<bool>(*this, "invert_right");
    rpm_scale_     = GetParam<int>(*this, "rpm_scale");
    min_abs_rpm_command_ = GetParam<double>(*this, "min_abs_rpm_command");

    // 피드백 설정
    bc_on_start_     = GetParam<bool>(*this, "main_data_broadcast_on_start");
    req_monitor_id_  = GetParamUint8<uint8_t>(*this, "req_monitor_id");
    use_polling_fb_  = GetParam<bool>(*this, "use_polling_feedback");
    request_hz_      = GetParam<double>(*this, "request_hz");
    tx_rate_hz_      = GetParam<double>(*this, "tx_rate_hz");

    // 오돔/TF/조인트
    base_frame_id_   = GetParam<std::string>(*this, "base_frame_id");
    odom_frame_id_   = GetParam<std::string>(*this, "odom_frame_id");
    publish_tf_      = GetParam<bool>(*this, "publish_tf");
    loop_hz_         = GetParam<double>(*this, "loop_hz");
    wheel_radius_    = GetParam<double>(*this, "wheel_radius");
    wheel_length_    = GetParam<double>(*this, "wheel_length");
    left_joint_name_ = GetParam<std::string>(*this, "left_joint_name");
    right_joint_name_= GetParam<std::string>(*this, "right_joint_name");

    // 피드백 끊김 시 추정 사용
    use_cmd_fallback_for_odom_ = GetParam<bool>(*this, "odom_use_cmd_if_no_rx");
    rx_stale_sec_              = GetParam<double>(*this, "rx_stale_sec");

    RCLCPP_INFO(get_logger(),
      "Params loaded (YAML only). port=%s baud=%d rmid=%u pcid=%u id=%u loop=%.1fHz",
      device_.c_str(), baudrate_, rmid_, pcid_, fixed_id_, loop_hz_);
  }

  // ========= 시리얼 =========
  void ConfigureSerial()
  {
    SerialCom::Params p;
    p.device      = device_;
    p.baudrate    = static_cast<unsigned int>(baudrate_);
    p.pc_id       = pcid_;
    p.mdt_id      = MID_MDT;
    p.mdui_id     = MID_MDUI;
    p.allowed_ids = { fixed_id_, ID_ALL };
    serial_.SetParams(p);

    if (!serial_.OpenPort()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s", device_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial port %s opened", device_.c_str());
    }

    if (bc_on_start_) {
      uint8_t cmd = CMD_PNT_MAIN_DATA_BC_ON;
      SendRaw(PID_COMMAND, &cmd, 1);
      RCLCPP_INFO(get_logger(), "Sent PID_COMMAND=CMD_PNT_MAIN_DATA_BC_ON");
    }
  }

  // ========= ROS I/F =========
  void SetupRos()
  {
    // 구독은 단순히 "마지막 명령"만 갱신
    wheel_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        "/wheel_speeds_cmd", rclcpp::QoS(20),
        std::bind(&SerialBridgeNode::OnWheelCmd, this, std::placeholders::_1));

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 20);
    odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    if (publish_tf_) tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // RX 폴링(아주 가볍게, 5 ms)
    rx_timer_ = create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&SerialBridgeNode::PollSerial, this));

    // TX는 주기 제한(기본 50 Hz) - 남발 방지
    const auto tx_period = std::chrono::duration<double>(1.0 / std::max(1.0, tx_rate_hz_));
    tx_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(tx_period),
        std::bind(&SerialBridgeNode::TickTx, this));

    // 오돔 적분
    const auto odom_period = std::chrono::duration<double>(1.0 / std::max(1.0, loop_hz_));
    odom_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(odom_period),
        std::bind(&SerialBridgeNode::IntegrateAndPublish, this));

    // (선택) 폴링 피드백 모드
    if (use_polling_fb_) {
      const auto req_period = std::chrono::duration<double>(1.0 / std::max(1.0, request_hz_));
      req_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(req_period),
          std::bind(&SerialBridgeNode::RequestFeedbackOnce, this));
    }
  }

  // ========= 수학 유틸 =========
  static inline double RpmToRadPerSec(double rpm) { return rpm * (2.0 * M_PI) / 60.0; }
  static inline double RadPerSecToRpm(double rad) { return rad * 60.0 / (2.0 * M_PI); }

  int16_t ToInt16Rpm(double omega_rad_s, bool invert) const
  {
    double rpm = RadPerSecToRpm(omega_rad_s);
    if (invert) rpm = -rpm;

    if (min_abs_rpm_command_ > 0.0 && std::fabs(rpm) > 0.0 && std::fabs(rpm) < min_abs_rpm_command_) {
      rpm = std::copysign(min_abs_rpm_command_, rpm);
    }

    const double scaled = rpm * static_cast<double>(rpm_scale_);
    const double clamped = std::clamp(scaled, -32768.0, 32767.0);
    return static_cast<int16_t>(std::lrint(clamped));
  }

  // ========= 저수준 TX =========
  void SendRaw(uint8_t pid, const void* data, uint8_t len)
  {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    serial_.SendPacket(rmid_, pcid_, fixed_id_, pid, p, len);
  }

  void SendVelCmd(double left_rad_s, double right_rad_s)
  {
    PntVelCmdPayload pl{};
    pl.enable_id1     = 1;
    pl.rpm_id1        = ToInt16Rpm(left_rad_s,  invert_left_);
    pl.enable_id2     = 1;
    pl.rpm_id2        = ToInt16Rpm(right_rad_s, invert_right_);
    pl.req_monitor_id = req_monitor_id_;
    SendRaw(PID_PNT_VEL_CMD, &pl, static_cast<uint8_t>(sizeof(PntVelCmdPayload)));
  }

  // ========= 콜백/타이머 =========
  void OnWheelCmd(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    // 마지막 명령만 저장 — 송신은 주기 타이머에서
    last_cmd_left_rad_s_  = msg->x;
    last_cmd_right_rad_s_ = msg->y;
    last_cmd_time_ = this->get_clock()->now();
  }

  void TickTx()
  {
    // 주기적으로 현재 목표 속도 전송
    SendVelCmd(last_cmd_left_rad_s_, last_cmd_right_rad_s_);
  }

  void RequestFeedbackOnce()
  {
    const uint8_t req_pid = PID_PNT_MAIN_DATA;
    SendRaw(PID_REQ_PID_DATA, &req_pid, 1);
  }

  void PollSerial()
  {
    serial_.ReadAndParse([this](const SerialCom::Frame& f){
      if (f.pid != PID_PNT_MAIN_DATA || f.length != sizeof(PntMainData)) return;

      PntMainData md{};
      std::memcpy(&md, f.data.data(), sizeof(PntMainData));

      const double l_rpm = static_cast<double>(md.rpm_id1) / static_cast<double>(rpm_scale_);
      const double r_rpm = static_cast<double>(md.rpm_id2) / static_cast<double>(rpm_scale_);

      // RX에도 극성 동일 적용
      last_left_rad_s_rx_  = (invert_left_  ? -1.0 : 1.0) * RpmToRadPerSec(l_rpm);
      last_right_rad_s_rx_ = (invert_right_ ? -1.0 : 1.0) * RpmToRadPerSec(r_rpm);
      last_rx_time_ = this->get_clock()->now();

      // JointState (RX 기반)
      sensor_msgs::msg::JointState js;
      js.header.stamp = last_rx_time_;
      js.name     = { left_joint_name_, right_joint_name_ };
      js.velocity = { last_left_rad_s_rx_, last_right_rad_s_rx_ };
      joint_pub_->publish(js);
    });
  }

  void IntegrateAndPublish()
  {
    const auto tnow = this->get_clock()->now();
    double dt = (tnow - last_time_).seconds();
    if (dt <= 0.0) dt = 1.0 / std::max(1.0, loop_hz_);
    last_time_ = tnow;

    // 속도 선택: RX가 신선하면 RX, 아니면 (옵션) 명령 기반 추정
    const bool rx_fresh = (tnow - last_rx_time_).seconds() <= rx_stale_sec_;
    const double wl = rx_fresh ? last_left_rad_s_rx_  : (use_cmd_fallback_for_odom_ ? last_cmd_left_rad_s_  : 0.0);
    const double wr = rx_fresh ? last_right_rad_s_rx_ : (use_cmd_fallback_for_odom_ ? last_cmd_right_rad_s_ : 0.0);

    // v = r/2 (ωL + ωR), w = r/L (ωR - ωL)
    const double v = (wheel_radius_ * 0.5) * (wl + wr);
    const double w = (wheel_length_ > 0.0) ? (wheel_radius_ / wheel_length_) * (wr - wl) : 0.0;

    x_     += v * std::cos(theta_) * dt;
    y_     += v * std::sin(theta_) * dt;
    theta_ += w * dt;

    // Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = tnow;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id  = base_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x  = v;
    odom.twist.twist.linear.y  = 0.0;
    odom.twist.twist.angular.z = w;

    // 간단 공분산
    for (double &c : odom.pose.covariance) c = 0.0;
    odom.pose.covariance[0]  = 0.05;
    odom.pose.covariance[7]  = 0.05;
    odom.pose.covariance[35] = 0.10;
    for (double &c : odom.twist.covariance) c = 0.0;
    odom.twist.covariance[0]  = 0.05;
    odom.twist.covariance[35] = 0.10;

    odom_pub_->publish(odom);

    // TF
    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = tnow;
      tf_msg.header.frame_id = odom_frame_id_;
      tf_msg.child_frame_id  = base_frame_id_;
      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf_msg);
    }

    // RX 오래 끊기면 경고(초당 1회)
    if (!rx_fresh && use_cmd_fallback_for_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No RX for %.2fs → using CMD fallback for odom.",
        (tnow - last_rx_time_).seconds());
    }
  }

private:
  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wheel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   joint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>               tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr rx_timer_, tx_timer_, odom_timer_, req_timer_;

  // Serial
  SerialCom serial_;

  // 파라미터/상태
  std::string device_;
  int         baudrate_;
  uint8_t     rmid_, pcid_, fixed_id_;
  bool        invert_left_, invert_right_;
  int         rpm_scale_;
  double      min_abs_rpm_command_;
  bool        bc_on_start_;
  uint8_t     req_monitor_id_;
  bool        use_polling_fb_;
  double      request_hz_;
  double      tx_rate_hz_;

  std::string base_frame_id_, odom_frame_id_;
  bool        publish_tf_;
  double      loop_hz_;
  double      wheel_radius_, wheel_length_;
  std::string left_joint_name_, right_joint_name_;

  bool        use_cmd_fallback_for_odom_;
  double      rx_stale_sec_;

  // 속도/자세/시간 상태
  double last_left_rad_s_rx_  = 0.0;
  double last_right_rad_s_rx_ = 0.0;

  double last_cmd_left_rad_s_  = 0.0;
  double last_cmd_right_rad_s_ = 0.0;

  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time last_rx_time_;
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
