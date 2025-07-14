#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import struct
import math

class ModbusReadNode(Node):
    def __init__(self):
        super().__init__('modbus_read_node')
        self.ser = None
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.retry_count = 3
        self.timeout = 1.0
        # Robot parameters
        self.wheel_diameter = 0.1651  # meters
        self.wheel_circumference = math.pi * self.wheel_diameter  # ~0.5184 meters
        self.counts_per_revolution = 16192  # Assuming 1000 pulses/rev with 4x encoding

        # Try to open serial port
        self._open_serial_port()

        # Create a timer to read registers every 500ms
        self.timer = self.create_timer(0.5, self.read_modbus_registers)

    def _open_serial_port(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            self.get_logger().info(f"Serial port {self.port} opened successfully")
            # Flush input/output buffers to clear any stale data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {str(e)}")
            self.ser = None

    def _calculate_crc(self, data):
        """Calculate Modbus RTU CRC-16."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)  # Little-endian CRC

    def read_error_status(self):
        """Read error codes from registers 20A5h (left) and 20A6h (right)."""
        if not self._check_serial():
            return None, None

        command = bytes([0x01, 0x03, 0x20, 0xA5, 0x00, 0x02]) + self._calculate_crc([0x01, 0x03, 0x20, 0xA5, 0x00, 0x02])
        try:
            self.ser.write(command)
            response = self.ser.read(9)  # Expect 9 bytes: 1 addr + 1 func + 1 count + 4 data + 2 CRC
            if len(response) == 9:
                left_error = int.from_bytes(response[3:5], byteorder='big', signed=False)
                right_error = int.from_bytes(response[5:7], byteorder='big', signed=False)
                self.get_logger().info(f"Left Motor Error Code: {hex(left_error)}, Right Motor Error Code: {hex(right_error)}")
                return left_error, right_error
            else:
                self.get_logger().warn(f"Invalid error status response length: {len(response)} bytes")
                return None, None
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error reading error status: {str(e)}")
            return None, None

    def _check_serial(self):
        """Check if serial port is open, attempt to reopen if closed."""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port is not open, attempting to reopen")
            self._open_serial_port()
        return self.ser and self.ser.is_open

    def read_modbus_registers(self):
        """Read motor positions and calculate distances every 500ms."""
        if not self._check_serial():
            self.get_logger().error("Cannot read registers: Serial port is not open")
            return

        # Check error status first
        left_error, right_error = self.read_error_status()
        if left_error != 0 or right_error != 0:
            self.get_logger().error("Driver in error state, may not respond to position queries")
            return

        # Command to read actual positions (registers 20A7h to 20AAh, 4 registers)
        command = bytes([0x01, 0x03, 0x20, 0xA7, 0x00, 0x04]) + self._calculate_crc([0x01, 0x03, 0x20, 0xA7, 0x00, 0x04])
        
        for attempt in range(self.retry_count):
            try:
                self.get_logger().info(f"Sending command (attempt {attempt + 1}/{self.retry_count}): {command.hex()}")
                self.ser.reset_input_buffer()  # Clear input buffer before sending
                self.ser.write(command)
                
                # Read response (expect 13 bytes: 1 addr + 1 func + 1 count + 8 data + 2 CRC)
                response = self.ser.read(13)
                self.get_logger().info(f"Received response: {response.hex()}")
                
                if len(response) == 13:
                    # Verify response address and function code
                    if response[0] != 0x01 or response[1] != 0x03:
                        self.get_logger().warn(f"Invalid response: Address {response[0]:02x}, Function {response[1]:02x}")
                        continue
                    # Parse response: bytes 3-6 for left motor, 7-10 for right motor
                    left_position_high = int.from_bytes(response[3:5], byteorder='big', signed=True)
                    left_position_low = int.from_bytes(response[5:7], byteorder='big', signed=True)
                    right_position_high = int.from_bytes(response[7:9], byteorder='big', signed=True)
                    right_position_low = int.from_bytes(response[9:11], byteorder='big', signed=True)
                    
                    # Combine high and low 16 bits to form 32-bit signed position values
                    left_position = (left_position_high << 16) | (left_position_low & 0xFFFF)
                    right_position = (right_position_high << 16) | (right_position_low & 0xFFFF)
                    
                    # Convert positions to distance (meters)
                    left_distance = (left_position / self.counts_per_revolution) * self.wheel_circumference
                    right_distance = (right_position / self.counts_per_revolution) * self.wheel_circumference
                    
                    self.get_logger().info(
                        f"Left Motor: {left_position} counts, Distance: {left_distance:.4f} meters\n"
                        f"Right Motor: {right_position} counts, Distance: {right_distance:.4f} meters"
                    )
                    return
                else:
                    self.get_logger().warn(f"Invalid response length received: {len(response)} bytes")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error on attempt {attempt + 1}: {str(e)}")
                self._open_serial_port()  # Attempt to reopen serial port
            except Exception as e:
                self.get_logger().error(f"Unexpected error on attempt {attempt + 1}: {str(e)}")
            
            time.sleep(0.2)  # Wait before retrying
        self.get_logger().error("Failed to read registers after all retries")

    def __del__(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Serial port closed")
            except Exception as e:
                self.get_logger().error(f"Error closing serial port: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ModbusReadNode()
    try:
        rclpy.spin(node)  # Keep node running to process timer callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.__del__()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
