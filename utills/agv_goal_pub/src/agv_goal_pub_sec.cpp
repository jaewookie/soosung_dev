#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>

class AgvGoalPubSec : public rclcpp::Node {
public:
  AgvGoalPubSec() : Node("agv_goal_pub_sec") {
    // 퍼블리셔 생성
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    // 사용자로부터 입력받기
    double x, y;
    std::cout << "목표 x 좌표를 입력하세요: ";
    std::cin >> x;
    std::cout << "목표 y 좌표를 입력하세요: ";
    std::cin >> y;

    // 메시지 생성
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = this->get_clock()->now();
    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.position.z = 0.0;
    goal_msg.pose.orientation.w = 1.0;  // 기본 방향 설정

    // 메시지 퍼블리시
    RCLCPP_INFO(this->get_logger(), "Publishing goal at x: %f, y: %f", x, y);
    publisher_->publish(goal_msg);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvGoalPubSec>());
  rclcpp::shutdown();
  return 0;
}
