#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <arpa/inet.h>

// start buffer -> 67 -> end buffer
// vector[0]~~~~~~ => process_cal[0]
// read interupt / proccese
// serial port read interupt => 67 byte store

#define GRAV_ACC 9.80665;

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("serial_node"), buffer_index_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SerialNode::timer_callback, this));

    serial_port_ = open("/dev/ttyS2", O_RDWR);
    if (serial_port_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error opening serial port");
      return;
    }

    struct termios tty;
    if (tcgetattr(serial_port_, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
      return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_cflag |= CREAD | CLOCAL;

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
      return;
    }

    start_sequence_ = {0x3a, 0x01, 0x00, 0x09, 0x00, 0x38, 0x00};
  }

  ~SerialNode()
  {
    if (serial_port_ >= 0)
    {
      close(serial_port_);
    }
  }

private:
  void timer_callback()
  {
    unsigned char temp_buffer[256];
    int n = read(serial_port_, &temp_buffer, sizeof(temp_buffer));
    if (n > 0)
    {
      for (int i = 0; i < n; i++)
      {
        buffer_.push_back(temp_buffer[i]);
      }

      // print_hex(buffer_);
      process_buffer();
    }
  }

  void process_buffer()
  {
    while (buffer_.size() >= 67)
    {
      auto it = std::search(buffer_.begin(), buffer_.end(), start_sequence_.begin(), start_sequence_.end());
      // if (it != buffer_.end())
      // {
      //   RCLCPP_INFO(this->get_logger(), "Found subsequence at position: %ld", std::distance(buffer_.begin(), it));
      //   RCLCPP_INFO(this->get_logger(), "Subsequence starts at: %d", *it);
      // }
      // else
      // {
      //   RCLCPP_INFO(this->get_logger(), "Subsequence not found.");
      // }
      if (it != buffer_.end())
      {
        size_t start_index = std::distance(buffer_.begin(), it);
        // RCLCPP_INFO(this->get_logger(), "%d", start_index);
        if (buffer_.size() >= start_index + 67)
        {
          std::vector<unsigned char> chunk(buffer_.begin() + start_index, buffer_.begin() + start_index + 67);
          // print_hex(chunk);
          process_chunk(chunk);
          buffer_.erase(buffer_.begin(), buffer_.begin() + start_index + 67);
        }
        else
        {
          break;
        }
      }
      // else
      // {
      //   buffer_.erase(buffer_.begin(), buffer_.end() - start_sequence_.size() + 1);
      //   break;
      // }
    }
  }

  void print_hex(const std::vector<unsigned char> &data)
  {
    std::stringstream ss;
    ss << "Received data (hex)" << std::endl;
    ss << "------------------------------------" << std::endl;
    // for (int i = 46; i < 62; i++)
    // {
    //   ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
    //   if (i % 4 == 1)
    //   {
    //     ss << std::endl;
    //   }
    //   if (i % 16 == 13)
    //   {
    //     ss << "------------------------------------";
    //   }
    // }
    for (unsigned char byte : data)
    {
      ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  void process_chunk(const std::vector<unsigned char> &chunk)
  {
    if (chunk.size() >= 67)
    {
      std::stringstream sscs;

      uint16_t calculated_checksum = 0;
      for (int i = 1; i < 63; i++)
      {
        calculated_checksum += chunk[i];
        // sscs << "chunk[" << i << "]" << std::endl;
        // sscs << chunk[i] << std::endl;
      }
      // sscs << "calculated_checksum : " << calculated_checksum << std::endl;
      // RCLCPP_INFO(this->get_logger(), "%s", sscs.str().c_str());

      // uint16_t received_checksum = 0;
      // std::memcpy(&received_checksum, &chunk[63], sizeof(received_checksum));
      // received_checksum = ntohl(received_checksum);
      uint16_t received_checksum = ((chunk[64]) << 8) | (chunk[63]);

      // sscs << "received_checksum : " << received_checksum << std::endl;
      // sscs << std::string("--------------------------------------") << std::endl;

      // RCLCPP_INFO(this->get_logger(), "%s", sscs.str().c_str());

      if (calculated_checksum == received_checksum)
      {
        std::stringstream ss;
        sensor_msgs::msg::Imu imu_msg;

        // Extract timestamp (bytes 7 to 10)
        uint32_t timestamp;
        std::memcpy(&timestamp, &chunk[7], sizeof(timestamp));
        // timestamp = ntohl(timestamp); // Convert from network byte order to host byte order

        ss << "timestamp : " << timestamp << std::endl;
        ss << std::string("--------------------------------------") << std::endl;

        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        // Extract Calibrated_Accel_x (bytes 11 to 14)
        float accel_x;
        std::memcpy(&accel_x, &chunk[11], sizeof(accel_x));
        accel_x = accel_x * GRAV_ACC;
        // accel_x = ntohl(accel_x); // Convert from network byte order to host byte order
        // int32_t accel_x_int;
        // std::memcpy(&accel_x_int, &accel_x, sizeof(accel_x_int));

        // std::memcpy(&accel_x, &accel_x_int, sizeof(accel_x));

        ss << "accel_x : " << accel_x << std::endl;

        // Extract Calibrated_Accel_y (bytes 15 to 18)
        float accel_y;
        std::memcpy(&accel_y, &chunk[15], sizeof(accel_y));
        accel_y = accel_y*GRAV_ACC;
        // uint32_t accel_y_int;
        // std::memcpy(&accel_y_int, &accel_y, sizeof(accel_y_int));
        // accel_y_int = ntohl(accel_y_int); // Convert from network byte order to host byte order
        // std::memcpy(&accel_y, &accel_y_int, sizeof(accel_y));

        ss << "accel_y : " << accel_y << std::endl;

        // Extract Calibrated_Accel_z (bytes 19 to 22)
        float accel_z;
        std::memcpy(&accel_z, &chunk[19], sizeof(accel_z));
        accel_z = accel_z * GRAV_ACC;
        // uint32_t accel_z_int;
        // std::memcpy(&accel_z_int, &accel_z, sizeof(accel_z_int));
        // accel_z_int = ntohl(accel_z_int); // Convert from network byte order to host byte order
        // std::memcpy(&accel_z, &accel_z_int, sizeof(accel_z));

        ss << "accel_z : " << accel_z << std::endl;
        ss << std::string("--------------------------------------") << std::endl;

        imu_msg.linear_acceleration.x = accel_x;
        imu_msg.linear_acceleration.y = accel_y;
        imu_msg.linear_acceleration.z = accel_z;

        // Extract gyro_x (bytes 23 to 26)
        float gyro_x;
        std::memcpy(&gyro_x, &chunk[23], sizeof(gyro_x));
        gyro_x = dpsToW(gyro_x);
        // uint32_t gyro_x_int;
        // std::memcpy(&gyro_x_int, &gyro_x, sizeof(gyro_x_int));
        // gyro_x_int = ntohl(gyro_x_int); // Convert from network byte order to host byte order
        // std::memcpy(&gyro_x, &gyro_x_int, sizeof(gyro_x));

        ss << "gyro_x : " << gyro_x << std::endl;

        // Extract gyro_y (bytes 27 to 30)
        float gyro_y;
        std::memcpy(&gyro_y, &chunk[27], sizeof(gyro_y));
        gyro_y = dpsToW(gyro_y);
        // uint32_t gyro_y_int;
        // std::memcpy(&gyro_y_int, &gyro_y, sizeof(gyro_y_int));
        // gyro_y_int = ntohl(gyro_y_int); // Convert from network byte order to host byte order
        // std::memcpy(&gyro_y, &gyro_y_int, sizeof(gyro_y));

        ss << "gyro_y : " << gyro_y << std::endl;

        // Extract gyro_z (bytes 31 to 34)
        float gyro_z;
        std::memcpy(&gyro_z, &chunk[31], sizeof(gyro_z));
        gyro_z = dpsToW(gyro_z);
        // uint32_t gyro_z_int;
        // std::memcpy(&gyro_z_int, &gyro_z, sizeof(gyro_z_int));
        // gyro_z_int = ntohl(gyro_z_int); // Convert from network byte order to host byte order
        // std::memcpy(&gyro_z, &gyro_z_int, sizeof(gyro_z));

        ss << "gyro_z : " << gyro_z << std::endl;
        ss << std::string("--------------------------------------") << std::endl;

        imu_msg.angular_velocity.x = gyro_x;
        imu_msg.angular_velocity.y = gyro_y;
        imu_msg.angular_velocity.z = gyro_z;

        // // Extract Accel_x (bytes 35 to 26)
        // float lin_acc_x;
        // std::memcpy(&lin_acc_x, &chunk[34], sizeof(lin_acc_x));
        // uint32_t lin_acc_x_int;
        // std::memcpy(&lin_acc_x_int, &lin_acc_x, sizeof(lin_acc_x_int));
        // lin_acc_x_int = ntohl(lin_acc_x_int); // Convert from network byte order to host byte order
        // std::memcpy(&lin_acc_x, &lin_acc_x_int, sizeof(lin_acc_x));

        // ss << "lin_acc_x : " << lin_acc_x << std::endl;

        // // Extract Accel_y (bytes 39 to 30)
        // float lin_acc_y;
        // std::memcpy(&lin_acc_y, &chunk[38], sizeof(lin_acc_y));
        // uint32_t lin_acc_y_int;
        // std::memcpy(&lin_acc_y_int, &lin_acc_y, sizeof(lin_acc_y_int));
        // lin_acc_y_int = ntohl(lin_acc_y_int); // Convert from network byte order to host byte order
        // std::memcpy(&lin_acc_y, &lin_acc_y_int, sizeof(lin_acc_y));

        // ss << "lin_acc_y : " << lin_acc_y << std::endl;

        // // Extract Accel_z (bytes 43 to 34)
        // float lin_acc_z;
        // std::memcpy(&lin_acc_z, &chunk[42], sizeof(lin_acc_z));
        // uint32_t lin_acc_z_int;
        // std::memcpy(&lin_acc_z_int, &lin_acc_z, sizeof(lin_acc_z_int));
        // lin_acc_z_int = ntohl(lin_acc_z_int); // Convert from network byte order to host byte order
        // std::memcpy(&lin_acc_z, &lin_acc_z_int, sizeof(lin_acc_z));

        // ss << "lin_acc_z : " << lin_acc_z << std::endl;
        // ss << std::string("--------------------------------------") << std::endl;

        // Extract Quat_w (bytes 35 to 26)
        float quat_w;
        std::memcpy(&quat_w, &chunk[47], sizeof(quat_w));
        // uint32_t quat_w_int;
        // std::memcpy(&quat_w_int, &quat_w, sizeof(quat_w_int));
        // quat_w_int = ntohl(quat_w_int); // Convert from network byte order to host byte order
        // std::memcpy(&quat_w, &quat_w_int, sizeof(quat_w));

        ss << "quat_w : " << quat_w << std::endl;

        // Extract Quat_x (bytes 39 to 30)
        float quat_x;
        std::memcpy(&quat_x, &chunk[51], sizeof(quat_x));
        // uint32_t quat_x_int;
        // std::memcpy(&quat_x_int, &quat_x, sizeof(quat_x_int));
        // quat_x_int = ntohl(quat_x_int); // Convert from network byte order to host byte order
        // std::memcpy(&quat_x, &quat_x_int, sizeof(quat_x));

        ss << "quat_x : " << quat_x << std::endl;

        // Extract Quat_y (bytes 43 to 34)
        float quat_y;
        std::memcpy(&quat_y, &chunk[55], sizeof(quat_y));
        // uint32_t quat_y_int;
        // std::memcpy(&quat_y_int, &quat_y, sizeof(quat_y_int));
        // quat_y_int = ntohl(quat_y_int); // Convert from network byte order to host byte order
        // std::memcpy(&quat_y, &quat_y_int, sizeof(quat_y));

        ss << "quat_y : " << quat_y << std::endl;

        // Extract Quat_z (bytes 43 to 34)
        float quat_z;
        std::memcpy(&quat_z, &chunk[59], sizeof(quat_z));
        // uint32_t quat_z_int;
        // std::memcpy(&quat_z_int, &quat_z, sizeof(quat_z_int));
        // quat_z_int = ntohl(quat_z_int); // Convert from network byte order to host byte order
        // std::memcpy(&quat_z, &quat_z_int, sizeof(quat_z));

        ss << "quat_z : " << quat_z << std::endl;
        ss << std::string("--------------------------------------") << std::endl;

        imu_msg.orientation.w = quat_w;
        imu_msg.orientation.x = quat_x;
        imu_msg.orientation.y = quat_y;
        imu_msg.orientation.z = quat_z;

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        publisher_->publish(imu_msg);
      }
      else
      {
        // RCLCPP_WARN(this->get_logger(), "Checksum mismatch. Discarding data.");
      }
    }
  }

  float dpsToW(float dps){
    return dps*M_PI/180;
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int serial_port_;
  std::vector<unsigned char> buffer_;
  std::vector<unsigned char> start_sequence_;
  size_t buffer_index_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
