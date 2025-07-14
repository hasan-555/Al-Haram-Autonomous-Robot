#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class DecimalToHexNode(Node):
    def __init__(self):
        super().__init__('decimal_to_hex_node')
        
        # Example: Convert 100 to "0x00 0x64"
        decimal_number = 45
        hex_str = self.decimal_to_16bit_hex(decimal_number)
        self.get_logger().info(f"Decimal {decimal_number} → 16-bit Hex: {hex_str}")

    def decimal_to_16bit_hex(self, num):
        """
        Converts a decimal number to a 16-bit hex string in "0x00 0x00" format.
        """
        if num < 0 or num > 65535:
            raise ValueError("Number must be between 0 and 65535 (16-bit unsigned).")
        
        # Convert to 16-bit hex (4 digits) and split into two bytes
        hex_full = f"0x{num:04X}"  # Formats as "0x0064" for 100
        byte1 = hex_full[:4]        # "0x00"
        byte2 = "0x" + hex_full[4:] # "0x64"
        
        return f"{byte1} {byte2}"

def main(args=None):
    rclpy.init(args=args)
    node = DecimalToHexNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
