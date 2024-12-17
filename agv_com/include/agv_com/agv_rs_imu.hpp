#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <vector>

class AgvRsImu : public rclcpp::Node
{
public:
    AgvRsImu();
    ~AgvRsImu();

private:
    void serialReadLoop();
    bool validateFrame(const std::vector<uint8_t> &frame);
    void parseFrame(const std::vector<uint8_t> &frame);
    int configureSerialPort(const std::string &port_name, int baud_rate);

    int serial_fd_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};
