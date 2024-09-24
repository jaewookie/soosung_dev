#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class AgvGoalPub : public rclcpp::Node
{
public:
  AgvGoalPub() : Node("agv_goal_pub")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    get_input();
    publish_goal();
  }

private:
  void get_input()
  {
    // 사용자로부터 좌표 입력 받기
    std::cout << "Enter the x coordinate for the goal: ";
    std::cin >> x_;
    std::cout << "Enter the y coordinate for the goal: ";
    std::cin >> y_;
  }
  void publish_goal()
  {
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.stamp = this->now();
    message.header.frame_id = "map";

    // 입력 받은 목표 위치 설정
    message.pose.position.x = x_;
    message.pose.position.y = y_;
    message.pose.position.z = 0.0;

    // 회전 없이 목표지점 설정
    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published goal: x=%f, y=%f", message.pose.position.x, message.pose.position.y);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  float x_, y_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvGoalPub>());
  rclcpp::shutdown();
  return 0;
}
