#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class RobotDescriptionTopicPublisher : public rclcpp::Node
{
public:
  RobotDescriptionTopicPublisher()
  : rclcpp::Node("robot_description_topic_publisher")
  {
    this->declare_parameter<std::string>("robot_description", "");
    std::string urdf_xml = this->get_parameter("robot_description").as_string();

    if (urdf_xml.empty())
    {
      RCLCPP_WARN(this->get_logger(),
                  "Parameter 'robot_description' is empty. Trying to fetch from /robot_state_publisher …");
      urdf_xml = TryFetchFromRsp();
      if (urdf_xml.empty())
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to obtain URDF. Set param 'robot_description' or ensure /robot_state_publisher has it.");
      }
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();
    pub_ = this->create_publisher<std_msgs::msg::String>("/robot_description", qos);

    // 1회 즉시 발행
    if (!urdf_xml.empty())
    {
      std_msgs::msg::String msg;
      msg.data = urdf_xml;
      pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published /robot_description (size=%zu)", urdf_xml.size());
      cached_urdf_ = urdf_xml;
    }

    // 신규 구독자 대비 주기 재공지(2초마다)
    timer_ = this->create_wall_timer(2s, std::bind(&RobotDescriptionTopicPublisher::RepublishTimer, this));
  }

private:
  std::string TryFetchFromRsp()
  {
    // /robot_state_publisher 파라미터 서버에서 robot_description을 동기 방식으로 시도
    auto client = std::make_shared<rclcpp::SyncParametersClient>(this, "/robot_state_publisher");
    // 노드가 뜰 시간을 조금 기다림(최대 2초)
    const auto deadline = now() + rclcpp::Duration(2, 0);
    while (!client->wait_for_service(200ms))
    {
      if (now() > deadline) return "";
    }

    try
    {
      auto values = client->get_parameters({ "robot_description" });
      if (!values.empty())
      {
        return values.front().as_string();
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to fetch from /robot_state_publisher: %s", e.what());
    }
    return "";
  }

  void RepublishTimer()
  {
    if (cached_urdf_.empty()) return;
    std_msgs::msg::String msg;
    msg.data = cached_urdf_;
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string cached_urdf_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDescriptionTopicPublisher>());
  rclcpp::shutdown();
  return 0;
}
