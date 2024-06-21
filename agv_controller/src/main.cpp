#include "rclcpp/rclcpp.hpp"

#include "agv_controller/agv_drive.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto agvDrive = std::make_shared<AgvDrive>();

  rclcpp::spin(agvDrive);

  rclcpp::shutdown();

  return 0;
}
