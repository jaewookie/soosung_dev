import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusTcpClient
from std_msgs.msg import Int32
import time

class ModbusNode(Node):
    def __init__(self):
        super().__init__('modbus_node')
        self.declare_parameter('modbus_host', '192.168.100.50')
        self.declare_parameter('modbus_port', 5020)
        self.declare_parameter('register_address', 0)
        self.declare_parameter('register_count', 10)
        self.declare_parameter('publish_frequency', 1.0)
        # user_id : 255

        # 파라미터 읽기
        self.modbus_host = self.get_parameter('modbus_host').get_parameter_value().string_value
        self.modbus_port = self.get_parameter('modbus_port').get_parameter_value().integer_value
        self.register_address = self.get_parameter('register_address').get_parameter_value().integer_value
        self.register_count = self.get_parameter('register_count').get_parameter_value().integer_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Modbus 클라이언트 설정
        self.client = ModbusTcpClient(self.modbus_host, port=self.modbus_port)
        self.publisher_ = self.create_publisher(Int32, 'modbus_data', 10)

        # 타이머 생성
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.read_and_publish)

        # 연결 확인 및 재시도
        while not self.client.connect():
            self.get_logger().warn("Connection to Modbus server failed. Retrying in 5 seconds...")
            time.sleep(5)
        self.get_logger().info("Connected to Modbus server.")

    def read_and_publish(self):
        # Modbus 레지스터 읽기
        response = self.client.read_holding_registers(self.register_address, self.register_count)
        if not response.isError():
            value = response.registers[0]
            msg = Int32()
            msg.data = value
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published value: {value}")
        else:
            self.get_logger().error("Failed to read Modbus registers")

    def destroy_node(self):
        self.client.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    modbus_node = ModbusNode()
    rclpy.spin(modbus_node)
    modbus_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
