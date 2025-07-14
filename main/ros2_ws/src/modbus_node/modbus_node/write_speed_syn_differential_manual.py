#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time

class DiffDriveModbusNode(Node):
    def __init__(self):
        super().__init__('diff_drive_modbus_node')
        
        # Robot parameters
        self.wheelbase = 0.38  # meters
        self.wheel_diameter = 0.1651  # meters
        self.wheel_circumference = 3.1416 * self.wheel_diameter  # meters
        
        # Serial port setup for Modbus RTU
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.get_logger().info("Serial port opened successfully")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        # Send enable command on initialization
        self.send_enable_command()

        # Example inputs for testing
        v = 20.0  # cm/s
        w = 0.0  # mrad/s
        self.process_velocity_command(v, w)

    def decimal_to_16bit_hex(self, num):
        """
        Converts a decimal number to a 16-bit hex string in '0x00 0x00' format.
        Handles negative numbers for signed 16-bit integers (-32768 to 32767).
        """
        if num < -32768 or num > 32767:
            raise ValueError("Number must be between -32768 and 32767 (16-bit signed).")
        
        # Handle negative numbers using two's complement
        if num < 0:
            num = 0x10000 + num  # Convert to unsigned 16-bit equivalent
        
        # Convert to 16-bit hex (4 digits) and split into two bytes
        hex_full = f"{num:04X}"
        byte1 = f"0x{hex_full[:2]}"  # High byte
        byte2 = f"0x{hex_full[2:]}"  # Low byte
        
        return byte1, byte2

    def calculate_crc16(self, data):
        """
        Calculates CRC-16 for a list of bytes using Modbus RTU polynomial.
        """
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

    def send_enable_command(self):
        """
        Sends the enable command to the ZLAC8015D driver.
        """
        if not self.ser or not self.ser.is_open:
            self.get_logger().error("Serial port not available")
            return
        
        # Enable command: 01 06 20 0E 00 08 E2 0F
        command = [0x01, 0x06, 0x20, 0x0E, 0x00, 0x08, 0xE2, 0x0F]
        try:
            self.ser.write(bytes(command))
            response = self.ser.read(8)  # Expect 8 bytes response
            response_hex = ' '.join(f'{b:02X}' for b in response)
            self.get_logger().info(f"Enable command sent. Response: {response_hex}")
            time.sleep(0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send enable command: {e}")

    def process_velocity_command(self, v, w):
        """
        Converts differential drive velocities to Modbus RTU command for ZLAC8015D.
        v: linear velocity in cm/s
        w: angular velocity in mrad/s
        """
        if not self.ser or not self.ser.is_open:
            self.get_logger().error("Serial port not available")
            return

        # Convert inputs: v (cm/s to m/s), w (mrad/s to rad/s)
        v_ms = v / 100.0  # cm/s to m/s
        w_rad = w / 1000.0  # mrad/s to rad/s

        # Differential drive kinematics
        v_left = v_ms - (w_rad * self.wheelbase / 2.0)  # m/s
        v_right = v_ms + (w_rad * self.wheelbase / 2.0)  # m/s

        # Convert to RPM: RPM = (v / circumference) * 60
        left_rpm = (v_left / self.wheel_circumference) * 60.0
        right_rpm = (v_right / self.wheel_circumference) * 60.0

        # Round to nearest integer and ensure within valid range (-3000 to 3000 RPM)
        left_rpm = (-1) * int(round(max(min(left_rpm, 3000), -3000)))
        right_rpm = int(round(max(min(right_rpm, 3000), -3000)))

        # Convert RPM to 16-bit hex
        left_high, left_low = self.decimal_to_16bit_hex(left_rpm)
        right_high, right_low = self.decimal_to_16bit_hex(right_rpm)

        # Construct Modbus RTU command for synchronous velocity control
        command = [
            0x01,  # Driver Address
            0x10,  # Function Code (Write Multiple Registers)
            0x20,  # Start Address High (0x2088)
            0x88,  # Start Address Low
            0x00,  # Number of Registers High
            0x02,  # Number of Registers Low (2 registers: 0x2088, 0x2089)
            0x04,  # Number of Bytes (4 bytes for two 16-bit registers)
            int(left_high, 16),  # Left velocity high byte
            int(left_low, 16),   # Left velocity low byte
            int(right_high, 16), # Right velocity high byte
            int(right_low, 16)   # Right velocity low byte
        ]

        # Calculate CRC
        crc = self.calculate_crc16(command)
        crc_low = crc & 0xFF
        crc_high = (crc >> 8) & 0xFF

        # Append CRC to command
        command.extend([crc_low, crc_high])

        # Send the command
        try:
            command_hex = ' '.join(f'{b:02X}' for b in command)
            self.ser.write(bytes(command))
            response = self.ser.read(8)  # Expect 8 bytes response
            response_hex = ' '.join(f'{b:02X}' for b in response)
            self.get_logger().info(
                f"Input: v={v:.2f} cm/s, w={w:.2f} mrad/s\n"
                f"Left Motor Velocity: {left_rpm:.2f} RPM ({left_high} {left_low})\n"
                f"Right Motor Velocity: {right_rpm:.2f} RPM ({right_high} {right_low})\n"
                f"Modbus RTU Command: {command_hex}\n"
                f"Response: {response_hex}"
            )
            time.sleep(0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send velocity command: {e}")

    def __del__(self):
        """
        Cleanup: Close the serial port when the node is destroyed.
        """
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Serial port closed")
            except Exception as e:
                self.get_logger().error(f"Failed to close serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveModbusNode()
    try:
        rclpy.spin_once(node)  # Run once to process the logger
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.__del__()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
