#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class CRCCalculatorNode(Node):
    def __init__(self):
        super().__init__('crc_calculator_node')
        self.input_sequence = [0x01, 0x10, 0x20, 0x08, 0x00, 0x32, 0x00, 0x32]
        self.calculate_and_print_crc()

    def calculate_crc16(self, data):
        crc = 0xFFFF
        polynomial = 0xA001
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= polynomial
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def calculate_and_print_crc(self):
        crc = self.calculate_crc16(self.input_sequence)
        low_byte = crc & 0xFF
        high_byte = (crc >> 8) & 0xFF
        self.get_logger().info(f"Input sequence: {' '.join(f'{b:02X}' for b in self.input_sequence)}")
        self.get_logger().info(f"CRC-16: {low_byte:02X} {high_byte:02X}")

def main(args=None):
    rclpy.init(args=args)
    node = CRCCalculatorNode()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
