#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>

class ModbusNode : public rclcpp::Node
{
public:
    ModbusNode() : Node("modbus_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Modbus communication...");

        // Modbus 설정
        ctx_ = modbus_new_tcp("192.168.100.50", 5020); // IP 주소 및 포트 설정

        int rc = modbus_set_slave(ctx_, 255);

        if (ctx_ == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to create the libmodbus context.");
            return;
        }

        if (modbus_connect(ctx_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", modbus_strerror(errno));
            modbus_free(ctx_);
            return;
        }

        // while (true)
        // {
        //     if (modbus_connect(ctx_) == -1)
        //     {
        //         std::cerr << i << "Connection failed: " << modbus_strerror(errno) << std::endl;
        //         // Wait for 2 seconds before retrying
        //         i++;
        //         std::this_thread::sleep_for(std::chrono::milliseconds(600));
        //         // modbus_free(ctx_);
        //     }
        //     else
        //     {
        //         std::cout << "Connected successfully to Modbus server" << std::endl;
        //         break;
        //     }
        // }

        // int register_address = 101;  // 쓸 레지스터 주소
        // uint16_t value_to_write = 1; // 쓸 값

        // if (modbus_write_register(ctx_, register_address, value_to_write) == -1)
        // {
        //     std::cerr << "Failed to write register: " << modbus_strerror(errno) << std::endl;
        //     modbus_close(ctx_);
        //     modbus_free(ctx_);
        //     return;
        // }

        // // 예제: 레지스터 읽기
        // uint16_t registers[10];

        // int rrc = modbus_read_registers(ctx_, 1, 5, registers);

        // if (rrc == -1)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to read register: %s", modbus_strerror(errno));
        // }
        // else
        // {
        //     for (int i = 0; i < rrc; ++i)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Register %d: %d", i, registers[i]);
        //     }
        // }

        // if (rc == -1)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to set Slave: %s", modbus_strerror(errno));
        // }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "Set Slave successfully.");
        //     if (rrc == -1)
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to read register: %s", modbus_strerror(errno));
        //     }
        //     else
        //     {
        //         for (int i = 0; i < rrc; ++i)
        //         {
        //             RCLCPP_INFO(this->get_logger(), "Register %d: %d", i, registers[i]);
        //         }
        //     }
        // }

        // Modbus 연결 종료
        modbus_close(ctx_);
        modbus_free(ctx_);
    }

private:
    modbus_t *ctx_;
    int i = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModbusNode>());
    rclcpp::shutdown();
    return 0;
}
