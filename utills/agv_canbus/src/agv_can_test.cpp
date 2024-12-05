#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

using namespace std::chrono_literals;

class CanNode : public rclcpp::Node
{
public:
  CanNode() : Node("can_node")
  {
    // CAN 메시지를 발행하기 위한 퍼블리셔 생성
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 10);

    // CAN 메시지를 수신하기 위한 서브스크립션 생성
    subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      "can_rx", 10, std::bind(&CanNode::can_callback, this, std::placeholders::_1));

    // 주기적으로 CAN 메시지를 보내기 위한 타이머 생성
    timer_ = this->create_wall_timer(
      500ms, std::bind(&CanNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = can_msgs::msg::Frame();
    message.id = 0x123;  // CAN ID
    message.dlc = 8;     // 데이터 길이
    message.data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};  // 데이터

    RCLCPP_INFO(this->get_logger(), "Publishing CAN message");
    publisher_->publish(message);
  }

  void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received CAN message - ID: 0x%X, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                msg->id,
                msg->data[0], msg->data[1], msg->data[2], msg->data[3],
                msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}
