import rclpy
from rclpy.node import Node
import serial

class MyNode(Node):
    def __init__(self):
        super().__init__('zlac8015d_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust port as needed
        # Define the command (example: read error codes from registers 20A5h and 20A6h)
        self.command = bytes([0x01, 0x03, 0x20, 0x08, 0x00, 0x02, 0x4E, 0x09])

    def _check_serial(self):
        return self.ser.is_open

    def send_command_and_print_response(self):
        if not self._check_serial():
            self.get_logger().error("Serial port is not open.")
            return

        func = self.command[1]
        if func == 0x03:
            reg_count = int.from_bytes(self.command[4:6], 'big')
            response_length = 5 + 2 * reg_count
        elif func in [0x06, 0x10]:
            response_length = 8
        else:
            self.get_logger().error(f"Unsupported function code: {hex(func)}")
            return

        try:
            self.ser.write(self.command)
            response = self.ser.read(response_length)
            if len(response) == response_length:
                hex_response = ' '.join(f'{b:02X}' for b in response)
                self.get_logger().info(f"Response: {hex_response}")
            else:
                self.get_logger().warn(f"Expected {response_length} bytes, but received {len(response)} bytes")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        node.send_command_and_print_response()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
