#include <memory>
#include <string>
#include <cmath>
#include <cstdint>
#include <vector>
#include <cstring>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "tutorial_mobile_robot/serial_com.hpp"

// ====== 예전 코드와 동일한 프로토콜 상수 ======
static constexpr uint8_t ID_ALL = 0xFE;

static constexpr uint8_t MID_MDUI = 184;  // 수신측(계기) ID (참고)
static constexpr uint8_t MID_MDT  = 183;  // 제어기(모터 드라이버) 쪽
static constexpr uint8_t MID_PC   = 172;  // PC 쪽

// PID
enum : uint8_t {
  PID_REQ_PID_DATA     = 4,
  PID_COMMAND          = 10,
  PID_PNT_VEL_CMD      = 207, // 모터 2개 속도 명령(동일 패킷)
  PID_PNT_MAIN_DATA    = 210, // 피드백(속도, 전류, 상태 등)
};

// COMMAND 코드
enum : uint8_t {
  CMD_PNT_MAIN_DATA_BC_ON = 61, // PID_PNT_MAIN_DATA 방송 ON
};

// 요청 타입(관례)
static constexpr uint8_t REQUEST_PNT_MAIN_DATA = 2;

// PID_PNT_VEL_CMD 페이로드(합계 7바이트, 리틀엔디안)
#pragma pack(push,1)
struct PntVelCmdPayload {
  uint8_t  enable_id1;
  int16_t  rpm_id1;     // LSB first
  uint8_t  enable_id2;
  int16_t  rpm_id2;     // LSB first
  uint8_t  req_monitor_id; // 0 또는 2(REQUEST_PNT_MAIN_DATA)
};
#pragma pack(pop)

// PID_PNT_MAIN_DATA(210) 수신 구조체(네 예전 코드와 동일)
#pragma pack(push,1)
struct MotorStateBits {
  uint8_t Alarm     : 1;
  uint8_t CtrlFail  : 1;
  uint8_t OverVolt  : 1;
  uint8_t OverTemp  : 1;
  uint8_t OverLoad  : 1;
  uint8_t HallFail  : 1;
  uint8_t InvVel    : 1;
  uint8_t Stall     : 1;
};

union MotorState {
  uint8_t val;
  MotorStateBits bits;
};

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

class SerialBridgeNode : public rclcpp::Node
{
public:
  SerialBridgeNode() : Node("serial_bridge_node"), serial_()
  {
    // ★ 파라미터
    device_   = declare_parameter<std::string>("device", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 19200);

    // 프레임 헤더 필드 (옛 코드 레이아웃과 동일)
    rmid_     = static_cast<uint8_t>(declare_parameter<int>("rmid", MID_MDT)); // 수신측(드라이버) ID
    pcid_     = static_cast<uint8_t>(declare_parameter<int>("pcid", MID_PC));  // 송신측(PC) ID
    fixed_id_ = static_cast<uint8_t>(declare_parameter<int>("fixed_id", 1));   // 항상 1 또는 0xFE

    // 단위/극성
    invert_left_  = declare_parameter<bool>("invert_left",  true);
    invert_right_ = declare_parameter<bool>("invert_right", false);

    // 속도 변환: rad/s → rpm(*scale) → int16
    // (많은 구형 펌웨어가 rpm 정수 또는 rpm*10을 받음)
    rpm_scale_ = declare_parameter<int>("rpm_scale", 1);   // 1 또는 10 등
    min_abs_rpm_command_ = declare_parameter<double>("min_abs_rpm_command", 0.0); // 너무 작은 값 스냅

    // 방송 요청 옵션
    bc_on_start_ = declare_parameter<bool>("main_data_broadcast_on_start", true);
    req_monitor_id_ = static_cast<uint8_t>(declare_parameter<int>("req_monitor_id", REQUEST_PNT_MAIN_DATA));

    // 시리얼 설정
    SerialCom::Params p;
    p.device      = device_;
    p.baudrate    = static_cast<unsigned int>(baudrate_);
    p.pc_id       = pcid_;                  // [1]바이트
    p.mdt_id      = MID_MDT;                // 수신 측(검사용)
    p.mdui_id     = MID_MDUI;               // 수신 측(검사용)
    p.allowed_ids = { fixed_id_, ID_ALL };  // 예전 수신 파서 호환(고정 1 또는 FE)
    serial_.SetParams(p);

    if (!serial_.OpenPort()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s", device_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial port %s opened", device_.c_str());
    }

    // 방송 ON (옵션) — PID=10, DATA=[61]
    if (bc_on_start_) {
      uint8_t cmd = CMD_PNT_MAIN_DATA_BC_ON;
      SendRaw(PID_COMMAND, &cmd, 1);
      RCLCPP_INFO(get_logger(), "Sent PID_COMMAND(10) = CMD_PNT_MAIN_DATA_BC_ON(61)");
    }

    // 명령 구독: /wheel_speeds_cmd (x: left ω[rad/s], y: right ω[rad/s])
    wheel_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        "/wheel_speeds_cmd", rclcpp::QoS(10),
        std::bind(&SerialBridgeNode::OnWheelCmd, this, std::placeholders::_1));

    // 피드백 퍼블리셔
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // 수신 폴링
    poll_timer_ = create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&SerialBridgeNode::PollSerial, this));
  }

private:
  // rad/s → rpm(int16 * rpm_scale)
  int16_t ToInt16Rpm(double omega_rad_s, bool invert) const
  {
    double rpm = omega_rad_s * 60.0 / (2.0 * M_PI);
    if (invert) rpm = -rpm;

    // 너무 작은 값 스냅(0이 아니면 최소 rpm 보장)
    if (min_abs_rpm_command_ > 0.0 && std::fabs(rpm) > 0.0 && std::fabs(rpm) < min_abs_rpm_command_) {
      rpm = std::copysign(min_abs_rpm_command_, rpm);
    }

    const double scaled = rpm * static_cast<double>(rpm_scale_);
    // 안전 클램프
    const double s = std::clamp(scaled, -32768.0, 32767.0);
    return static_cast<int16_t>(std::lrint(s));
  }

  // 예전 프레임 그대로 송신 (RMID/PCID/ID=1/PID/LEN/DATA/CHK)
  void SendRaw(uint8_t pid, const void* data, uint8_t len)
  {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    serial_.SendPacket(
      rmid_,        // RMID(수신측)
      pcid_,        // PCID(송신측)
      fixed_id_,    // ID(항상 1 또는 FE)
      pid,
      p,
      len
    );
  }

  // 한 패킷으로 좌/우 속도 전송 (PID_PNT_VEL_CMD = 207)
  void SendVelCmd(double left_rad_s, double right_rad_s)
  {
    PntVelCmdPayload pl{};
    pl.enable_id1    = 1; // 항상 enable
    pl.rpm_id1       = ToInt16Rpm(left_rad_s,  invert_left_);
    pl.enable_id2    = 1;
    pl.rpm_id2       = ToInt16Rpm(right_rad_s, invert_right_);
    pl.req_monitor_id= req_monitor_id_; // 0 또는 2

    // 디버그 출력
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "TX PID=207 : enable(1,1) rpm(%d,%d) req=%u (scale=%d)",
      (int)pl.rpm_id1, (int)pl.rpm_id2, (unsigned)pl.req_monitor_id, rpm_scale_);

    SendRaw(PID_PNT_VEL_CMD, &pl, static_cast<uint8_t>(sizeof(PntVelCmdPayload)));
  }

  void OnWheelCmd(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    // msg->x = left ω[rad/s], msg->y = right ω[rad/s]
    SendVelCmd(msg->x, msg->y);
  }

  // 수신 폴링 → PID_PNT_MAIN_DATA(210) 해석 → /joint_states
  void PollSerial()
  {
    serial_.ReadAndParse([this](const SerialCom::Frame& f){
      if (f.pid == PID_PNT_MAIN_DATA && f.length == sizeof(PntMainData)) {
        PntMainData md{};
        std::memcpy(&md, f.data.data(), sizeof(PntMainData));

        // rpm(int16) → rad/s
        const double l_rpm = static_cast<double>(md.rpm_id1) / static_cast<double>(rpm_scale_);
        const double r_rpm = static_cast<double>(md.rpm_id2) / static_cast<double>(rpm_scale_);
        const double l_rad = l_rpm * (2.0 * M_PI) / 60.0;
        const double r_rad = r_rpm * (2.0 * M_PI) / 60.0;

        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        js.name     = {"left_wheel_joint", "right_wheel_joint"};
        js.velocity = { l_rad, r_rad };
        joint_pub_->publish(js);

        RCLCPP_DEBUG(get_logger(), "RX PID=210 : rpm(%d,%d) → rad/s(%.3f, %.3f)",
                     (int)md.rpm_id1, (int)md.rpm_id2, l_rad, r_rad);
      }
    });
  }

private:
  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wheel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   joint_pub_;
  rclcpp::TimerBase::SharedPtr                                  poll_timer_;

  // Serial
  SerialCom serial_;

  // 파라미터
  std::string device_;
  int         baudrate_;

  uint8_t     rmid_;        // = 183(default, MDT)
  uint8_t     pcid_;        // = 172(default, PC)
  uint8_t     fixed_id_;    // = 1(default)
  bool        invert_left_;
  bool        invert_right_;
  int         rpm_scale_;   // = 1 or 10
  double      min_abs_rpm_command_;
  bool        bc_on_start_;
  uint8_t     req_monitor_id_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
