import rclpy
from rclpy.node import Node
import serial
import time

class ModbusReadNode(Node):
    def __init__(self):
        super().__init__('modbus_read_node')
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
        except serial.SerialException:
            self.ser = None
            self.get_logger().error("Failed to open serial port /dev/ttyUSB0")
    
    def read_modbus_registers(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().error("Serial port is not open")
            return
        
        try:
            # Command to read actual velocities (registers 20ABh and 20ACh, 2 registers)
            command = bytes([0x01, 0x03, 0x20, 0xAB, 0x00, 0x02, 0xBE, 0x2B])
            self.ser.write(command)
            
            # Read response (expected: 9 bytes = 1 addr + 1 func + 1 byte count + 4 data bytes + 2 CRC)
            response = self.ser.read(9)
            if len(response) == 9:
                # Parse response: byte 3-6 contain the data (left and right velocities)
                left_velocity = int.from_bytes(response[3:5], byteorder='big', signed=True) * 0.1  # Unit: r/min
                right_velocity = int.from_bytes(response[5:7], byteorder='big', signed=True) * 0.1  # Unit: r/min
                self.get_logger().info(f"Left Motor Velocity: {left_velocity:.1f} RPM, Right Motor Velocity: {right_velocity:.1f} RPM")
            else:
                self.get_logger().warn("Invalid response length received")
                
            time.sleep(0.1)  # Small delay to avoid overwhelming the driver
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")

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
        node.read_modbus_registers()
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.__del__()
        rclpy.shutdown()

if __name__ == '__main__':
    main()