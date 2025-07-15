# Al-Haram-Autonomous-Robot

This repository contains the code for an autonomous robot developed for Masjid Al Haram, Saudi Arabia, to assist visitors and tourists. The project includes control systems, sensor fusion, and ROS2 Humble interfaces for motor and sensor integration, developed as part of a work project at SWB Syria Company, Aleppo, Syria, from April 2025 to June 2025.

## Project Overview

The goal of this project was to design an autonomous robot with natural language processing (NLP) capabilities to assist visitors at Masjid Al Haram. Key features include:
- PID-based torque and speed control for ZLLG80ASM250-L motors.
- Sensor fusion using Time-of-Flight (ToF), MPU6050, and BNO055 IMUs.
- Motion control via ZLAC8015D servo drivers over RS485.
- ROS2 Humble interfaces for motor and sensor integration.
- Simulation of kinematics and dynamics using Gazebo and Rviz2.
- Project management using Azure DevOps for milestone tracking.

## Repository Structure

The repository is organized as follows:

```
Al-Haram-Autonomous-Robot/
├── main/
│   ├── PID-Motor-Control/
│   │   ├── Big_Motor_PID/
│   │   │   ├── main.cpp                    # PID speed control for ZLLG80ASM250-L motors with MPU6050
│   │   │   ├── MotorDrive.cpp             # Motor control logic for differential drive
│   │   │   ├── MotorDrive.h               # Header file for MotorDrive class
│   │   ├── Prototpe_class/
│   │   │   ├── main.cpp                   # Class-based PID control with BNO055 IMU and WiFi
│   │   │   ├── MotorDrive.cpp             # Updated motor control logic for differential drive
│   │   │   ├── MotorDrive.h               # Header file for MotorDrive class
│   │   ├── Prototype_V2_With_TOF/
│   │   │   ├── main.cpp                   # Sensor fusion with ToF, BNO055 IMU, and OLED display
│   ├── ros2_ws/                           # ROS2 Humble workspace for ZLAC8015D servo driver communication
│   │   ├── src/
│   │   │   ├── modbus_node/
│   │   │   │   ├── read_distance.py                # Reads motor positions and calculates distances
│   │   │   │   ├── read_distance_pub_odom.py       # Reads motor positions, calculates distances, and publishes to /wheel/odom
│   │   │   │   ├── read_speed_values.py            # Reads motor velocities from ZLAC8015D
│   │   │   │   ├── reset_distance.py               # Resets motor position registers to zero
│   │   │   │   ├── send_raw_modbus_series.py       # Sends raw Modbus commands for testing
│   │   │   │   ├── write_speed_sync_sub_cmdvel.py  # Subscribes to /cmd_vel for differential drive control
│   │   │   │   ├── write_speed_syn_differential_manual.py # Manual differential drive velocity control
│   │   │   ├── crc_calculator/
│   │   │   │   ├── crc_calculator.py               # Calculates CRC-16 for a given Modbus sequence
│   │   │   │   ├── decimal_to_hex_node.py          # Converts decimal to 16-bit hex string
│   │   │   │   ├── request_data.py                 # Sends Modbus command and prints response
│   │   │   │   ├── send_modbus_series.py           # Sends a series of Modbus commands
├── README.md
```

### Component Details

1. **Big_Motor_PID**
   - **Purpose**: Implements PID-based speed control for ZLLG80ASM250-L motors using MPU6050 for IMU data and encoder feedback.
   - **Hardware**: ESP32 microcontroller, ZLLG80ASM250-L motors, MPU6050 IMU, ACS712 current sensor, encoders.
   - **Files**:
     - `main.cpp`: Implements PID speed control for left and right motors, calculates motor speeds, and logs data via serial output.
     - `MotorDrive.cpp`: Provides motor control logic for differential drive, including PWM control, direction setting, and distance/speed calculations using encoder data.
     - `MotorDrive.h`: Defines the `MotorDrive` class interface for motor control and encoder handling.
   - **Description**: Uses a PID controller to maintain target speeds (e.g., 8 cm/s for left, -8 cm/s for right) with encoder feedback. Supports JSON-based velocity commands for differential drive.

2. **Prototpe_class**
   - **Purpose**: A refactored, class-based implementation of PID speed control for ZLLG80ASM250-L motors, integrated with BNO055 IMU and WiFi connectivity.
   - **Hardware**: ESP32 microcontroller, ZLLG80ASM250-L motors, BNO055 IMU, encoders.
   - **Files**:
     - `main.cpp`: Implements PID control with BNO055 IMU calibration, WiFi connectivity (static IP), and JSON-based telemetry output (accelerometer and gyro data).
     - `MotorDrive.cpp`: Updated motor control logic for differential drive, supporting JSON velocity commands and encoder-based distance/speed calculations.
     - `MotorDrive.h`: Defines the `MotorDrive` class interface, same as in `Big_Motor_PID` but used with WiFi and IMU integration.
   - **Description**: Enhances `Big_Motor_PID` with WiFi (connects to "SWB" network) and BNO055 IMU for orientation data. Supports telnet for remote control and ElegantOTA for firmware updates.

3. **Prototype_V2_With_TOF**
   - **Purpose**: Implements sensor fusion with three Time-of-Flight (ToF) sensors, BNO055 IMU, and battery monitoring, with data displayed on an OLED screen.
   - **Hardware**: ESP32 microcontroller, BNO055 IMU, three ToF sensors (HC-SR04), SSD1306 OLED display, battery voltage monitoring circuit.
   - **Files**:
     - `main.cpp`: Reads data from ToF sensors, BNO055 IMU, and battery monitoring pins, performs sensor fusion, and outputs JSON telemetry (IMU, ToF distances, battery state) via serial.
   - **Description**: Uses three ToF sensors to detect obstacles (thresholds at 50 cm), calibrates BNO055 IMU for accurate accelerometer and gyro data, and monitors battery charge percentage and connection state. Data is logged in JSON format and can be displayed on an OLED.

4. **ros2_ws**
   - **Purpose**: ROS2 Humble workspace for motion control and sensor integration using ZLAC8015D servo drivers over RS485.
   - **Hardware**: ZLAC8015D servo drivers, RS485 communication interface.
   - **Description**: Contains two ROS2 packages: `modbus_node` and `crc_calculator`.
     - **modbus_node**:
       - `read_distance.py`: Reads motor positions from registers 20A7h-20AAh, calculates distances traveled by left and right wheels, and logs the results.
       - `read_distance_pub_odom.py`: Extends `read_distance.py` by publishing wheel distances to the `/wheel/odom` topic as a `geometry_msgs/Point` message (x for left, y for right).
       - `read_speed_values.py`: Reads motor velocities from registers 20ABh and 20ACh and logs them in RPM.
       - `reset_distance.py`: Resets motor position registers (2084h, 208Bh, 208Ch, 208Dh) to zero before reading positions and calculating distances.
       - `send_raw_modbus_series.py`: Sends a predefined series of raw Modbus commands for testing the ZLAC8015D driver.
       - `write_speed_sync_sub_cmdvel.py`: Subscribes to the `/cmd_vel` topic, converts linear and angular velocities to motor RPM, and sends synchronous velocity commands to the ZLAC8015D driver.
       - `write_speed_syn_differential_manual.py`: Sends manual differential drive velocity commands (e.g., 20 cm/s linear, 0 mrad/s angular) to the ZLAC8015D driver.
     - **crc_calculator**:
       - `crc_calculator.py`: Calculates CRC-16 for a given Modbus sequence and logs the result.
       - `decimal_to_hex_node.py`: Converts a decimal number to a 16-bit hex string in the format "0x00 0x00".
       - `request_data.py`: Sends a Modbus command (e.g., read registers 20A5h-20A6h for error codes) and logs the response.
       - `send_modbus_series.py`: Sends a series of predefined Modbus commands for testing, such as setting control parameters.

## Prerequisites

To run the code in this repository, ensure you have the following:

- **Hardware**:
  - ESP32 microcontroller
  - ZLLG80ASM250-L motors
  - ZLAC8015D servo drivers (with RS485 interface)
  - MPU6050 and BNO055 IMUs
  - Three HC-SR04 ToF sensors
  - SSD1306 OLED display (I2C)
  - ACS712 current sensor (for `Big_Motor_PID`)
  - RS485-to-USB adapter for serial communication
  - Battery voltage monitoring circuit
- **Software**:
  - [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
  - [Arduino IDE](https://www.arduino.cc/en/software) or PlatformIO for ESP32 firmware
  - Python 3.8+ for ROS2 nodes
  - [Gazebo](https://gazebosim.org/) and [Rviz2](https://github.com/ros2/rviz) for simulation
- **Dependencies**:
  - Install required ROS2 packages:
    ```bash
    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros-pkgs ros-humble-geometry-msgs
    ```
  - Python libraries:
    ```bash
    pip install pyserial
    ```
  - Arduino libraries: `Wire`, `MPU6050`, `Adafruit_BNO055`, `ESP32Encoder`, `ESPAsyncWebServer`, `ElegantOTA`, `ESPTelnet`, `ArduinoJson`, `Adafruit_GFX`, `Adafruit_SSD1306`

## Setup Instructions

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/<your-username>/Al-Haram-Autonomous-Robot.git
   cd Al-Haram-Autonomous-Robot
   ```

2. **Big_Motor_PID**:
   - Open the `Big_Motor_PID` folder in Arduino IDE or PlatformIO.
   - Connect the ESP32 to your computer.
   - Install required libraries: `ESP32Encoder`, `ArduinoJson`.
   - Upload the firmware to the ESP32.
   - Ensure MPU6050, ACS712, and encoders are connected (pins defined in `main.cpp`).

3. **Prototpe_class**:
   - Open the `Prototpe_class` folder in Arduino IDE or PlatformIO.
   - Install required libraries: `ESP32Encoder`, `ArduinoJson`, `Adafruit_BNO055`, `Adafruit_Sensor`, `ESPAsyncWebServer`, `ElegantOTA`, `ESPTelnet`.
   - Connect the ESP32 and upload the firmware.
   - Ensure BNO055 and encoders are connected (pins defined in `main.cpp`).
   - Configure WiFi credentials in `main.cpp` (`ap_ssid`, `ap_password`) and static IP settings if needed.

4. **Prototype_V2_With_TOF**:
   - Open the `Prototype_V2_With_TOF` folder in Arduino IDE or PlatformIO.
   - Install required libraries: `Adafruit_BNO055`, `Adafruit_Sensor`, `ArduinoJson`, `Adafruit_GFX`, `Adafruit_SSD1306`, `ESPAsyncWebServer`, `ElegantOTA`, `ESPTelnet`.
   - Connect the ESP32 and upload the firmware.
   - Ensure BNO055, three HC-SR04 ToF sensors, SSD1306 OLED, and battery monitoring circuit are connected (pins defined in `main.cpp`).

5. **ros2_ws**:
   - Initialize and build the ROS2 workspace:
     ```bash
     cd main/ros2_ws
     colcon build
     source install/setup.bash
     ```
   - Connect the ZLAC8015D servo drivers via an RS485-to-USB adapter to `/dev/ttyUSB0` (adjust port as needed).
   - Run individual nodes from the `modbus_node` or `crc_calculator` packages, e.g.:
     ```bash
     ros2 run modbus_node read_distance
     ros2 run crc_calculator crc_calculator
     ```

6. **Simulation**:
   - Launch Gazebo and Rviz2 for kinematics and dynamics validation:
     ```bash
     ros2 launch <package_name> gazebo_launch.py
     ros2 launch <package_name> rviz2_launch.py
     ```
   - Note: Simulation launch files are not included in this repository but can be created based on the robot’s URDF model.

## Usage

- **Motor Control**:
  - **Big_Motor_PID**: Run the firmware to test PID-based speed control (target speeds: 8 cm/s left, -8 cm/s right). Monitor motor speeds and current via serial output.
  - **Prototpe_class**: Run the firmware to test class-based PID control with BNO055 IMU data. Send JSON velocity commands (e.g., `{"m":{"v":15,"w":0}}`) via serial or telnet to control the robot. Monitor IMU and motor data via serial.

- **Sensor Fusion**:
  - **Prototype_V2_With_TOF**: Run the firmware to test sensor fusion with ToF sensors, BNO055 IMU, and battery monitoring. Monitor JSON output via serial for IMU data, ToF distances, and battery state. Obstacle detection triggers when any ToF sensor reads <50 cm. Data can be displayed on the OLED.

- **ROS2 Interface**:
  - **modbus_node**:
    - Run `read_distance.py` to log motor positions and calculated distances.
    - Run `read_distance_pub_odom.py` to publish wheel distances to `/wheel/odom` for odometry integration.
    - Run `read_speed_values.py` to monitor motor velocities.
    - Run `reset_distance.py` to reset motor position registers before starting navigation.
    - Run `send_raw_modbus_series.py` to test raw Modbus commands.
    - Run `write_speed_sync_sub_cmdvel.py` to control the robot via `/cmd_vel` topic messages.
    - Run `write_speed_syn_differential_manual.py` to test manual velocity commands.
  - **crc_calculator**:
    - Run `crc_calculator.py` to compute CRC-16 for a Modbus sequence.
    - Run `decimal_to_hex_node.py` to convert decimal numbers to 16-bit hex format.
    - Run `request_data.py` to send a Modbus command and log the response.
    - Run `send_modbus_series.py` to send a series of Modbus commands for testing.

- **Simulation**:
  - Validate the robot’s kinematics and dynamics in Gazebo/Rviz2 before hardware deployment.

## Contributing

Contributions are welcome! Please submit a pull request or open an issue to discuss improvements or bug fixes.

## License

This project is licensed under the MIT License.

## Contact

For questions or support, contact the project team at SWB Syria Company or open an issue in this repository.