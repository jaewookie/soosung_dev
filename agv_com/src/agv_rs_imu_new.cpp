// imu_parser.cpp
#include "agv_com/agv_rs_imu.hpp"

AgvRsImu::AgvRsImu()
    : Node("imu_parser")
{
    RCLCPP_INFO(this->get_logger(), "aaaaa");
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    RCLCPP_INFO(this->get_logger(), "bbbbbb");
    serial_fd_ = configureSerialPort("/dev/ttyS2", 115200);
    RCLCPP_INFO(this->get_logger(), "cccccc");
    if (serial_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        rclcpp::shutdown();
        return;
    }

    std::thread(&AgvRsImu::serialReadLoop, this).detach();
}

AgvRsImu::~AgvRsImu()
{
    if (serial_fd_ >= 0)
    {
        close(serial_fd_);
    }
}

int AgvRsImu::configureSerialPort(const std::string &port_name, int baud_rate)
{
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("Error opening serial port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("Error getting terminal attributes");
        close(fd);
        return -1;
    }

    // ?? ??
    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit data
    tty.c_iflag &= ~IGNBRK;                     // Ignore break processing
    tty.c_lflag = 0;                            // No signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // No remapping, no delays
    tty.c_cc[VMIN] = 1;                         // ?? ?? ??? ?
    tty.c_cc[VTIME] = 5;                        // ?? ???? (0.1?)

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No XON/XOFF software flow control
    tty.c_cflag |= (CLOCAL | CREAD);        // Enable receiver, local mode
    tty.c_cflag &= ~(PARENB | PARODD);      // No parity
    tty.c_cflag &= ~CSTOPB;                 // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                // No hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("Error setting terminal attributes");
        close(fd);
        return -1;
    }

    return fd;
}

void AgvRsImu::serialReadLoop()
{
    RCLCPP_INFO(this->get_logger(), "dddddd");
    std::vector<uint8_t> buffer(67);

    while (rclcpp::ok())
    {
        ssize_t bytes_read = read(serial_fd_, buffer.data(), buffer.size());
        RCLCPP_INFO(this->get_logger(), "%ld", buffer.data());
        if (bytes_read == 67)
        { // ??? 67???? ???? ??
            RCLCPP_INFO(this->get_logger(), "eeeeee");
            if (validateFrame(buffer))
            {
                parseFrame(buffer);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Invalid frame received");
            }
        }
        else if (bytes_read < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial read error");
            break;
        }
    }
}

bool AgvRsImu::validateFrame(const std::vector<uint8_t> &frame)
{
    if (frame[0] != 0x3A || frame[1] != 0x01 || frame[65] != 0x0D || frame[66] != 0x0A)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Frame header or footer mismatch. "
                     "Expected header: 3A 01, footer: 0D 0A, but got: "
                     "Header: %02X %02X, Footer: %02X %02X",
                     frame[0], frame[1], frame[65], frame[66]);
        return false;
    }

    uint16_t checksum = 0;
    for (size_t i = 1; i <= 62; ++i)
    {
        checksum += frame[i];
    }
    uint8_t checksum_low = checksum & 0xFF;         // ???? ?? ???
    uint8_t checksum_high = (checksum >> 8) & 0xFF; // ???? ?? ???

    if (checksum_low != frame[63] || checksum_high != frame[64])
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Checksum mismatch. "
                     "Calculated: %02X %02X, Received: %02X %02X",
                     checksum_low, checksum_high, frame[63], frame[64]);
        return false;
    }

    return true;
}

void AgvRsImu::parseFrame(const std::vector<uint8_t> &frame)
{
    // ?????
    uint32_t timestamp = frame[7] | (frame[8] << 8) | (frame[9] << 16) | (frame[10] << 24);

    // ??? (??????? ?)
    float accel_x = *reinterpret_cast<const float *>(&frame[11]);
    float accel_y = *reinterpret_cast<const float *>(&frame[15]);
    float accel_z = *reinterpret_cast<const float *>(&frame[19]);

    // ???
    float gyro_x = *reinterpret_cast<const float *>(&frame[23]);
    float gyro_y = *reinterpret_cast<const float *>(&frame[27]);
    float gyro_z = *reinterpret_cast<const float *>(&frame[31]);

    // ????
    float quat_w = *reinterpret_cast<const float *>(&frame[47]);
    float quat_x = *reinterpret_cast<const float *>(&frame[51]);
    float quat_y = *reinterpret_cast<const float *>(&frame[55]);
    float quat_z = *reinterpret_cast<const float *>(&frame[59]);

    // ROS2 ??? ?? ? ????
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.orientation.w = quat_w;
    imu_msg.orientation.x = quat_x;
    imu_msg.orientation.y = quat_y;
    imu_msg.orientation.z = quat_z;

    imu_msg.linear_acceleration.x = accel_x;
    imu_msg.linear_acceleration.y = accel_y;
    imu_msg.linear_acceleration.z = accel_z;

    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;

    imu_publisher_->publish(imu_msg);
}
int main(int argc, char *argv[])
{
    // ROS2 ???
    rclcpp::init(argc, argv);

    // IMUParser ?? ??
    auto agv_rs_imu_node = std::make_shared<AgvRsImu>();
    rclcpp::spin(agv_rs_imu_node);
    rclcpp::shutdown();
    return 0;
}