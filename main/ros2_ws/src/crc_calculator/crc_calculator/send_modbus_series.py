import rclpy
from rclpy.node import Node
import serial
import time

class ModbusNode(Node):
    def __init__(self):
        super().__init__('modbus_node')
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

    def send_hex_series(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            commands = [
             bytes([0x01, 0x10, 0x20, 0x08, 0x00, 0x32, 0x00, 0x32, 0x56, 0xBD])
            #   bytes([0x01, 0x06, 0x20, 0x0E, 0x00, 0x08, 0xE2, 0x0F]),
            #   bytes([0x01, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04, 0x00, 0x2D, 0x00, 0x2D, 0x33, 0xBC]) 
            ]
            for cmd in commands:
                self.ser.write(cmd)
                self.ser.read(8)
                time.sleep(0.1)
        except (serial.SerialException, Exception):
            pass

    def __del__(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()
    try:
        node.send_hex_series()
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.__del__()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
