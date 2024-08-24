# Lawn Tractor Embedded Code

This repository contains various embedded programs designed to run on different microcontrollers for controlling and monitoring a lawn tractor system. Each program is tailored to interact with specific hardware components and sensors, enabling functionalities such as motion control, sensor data acquisition, and communication with a ROS (Robot Operating System) environment.

## Table of Contents

1. [Project Overview](#project-overview)
2. [Hardware Overview](#hardware-overview)
3. [Program Descriptions](#program-descriptions)
    - [tractor_control](#tractor_control)
    - [left_speed](#leftSpeed)
    - [right_speed](#rightSpeed)
    - [radio_control](#tractor_control)
    - [BNO085_ROS_IMU_Publisher](#bno085_ros_imu_publisher)    
4. [Installation](#installation)
5. [Usage](#usage)
6. [Contributing](#contributing)
7. [License](#license)

## Project Overview

This project involves multiple embedded systems working together to perform various tasks related to the control and monitoring of a lawn tractor. Each system is developed using different microcontrollers, each dedicated to a specific function such as sensor data acquisition, motor control, or communication.

## Hardware Overview

The lawn tractor system comprises various hardware components, including microcontrollers, sensors, and communication modules. The main components include:

- **ROS Master Laptop**: The central unit that receives and processes data from various sensors.
- **Wheel Sensors**: Two microcontrollers provide speed and distance travelled data on a ROS topic by reading an AS5048 magnetic rotational sensor on each wheel.
- **Tractor Control Unit**: A microcontroller used for interfacing with sensors and communicating with the ROS environment.  Two primary control devices are a motor controller driving the steering and a servo controlling the tractor hydrostatic transmission.  This unit is using the Lilygo TTGO LoRa board http://www.lilygo.cn/prod_view.aspx?TypeId=50060&Id=1135&FId=t3:50060:3  This receives control instructions from radio control or cmd_vel and instructs the steering motor controller and transmission controller accordingly.
- **IMU Sensor**: An Adafruit Bosch BNO085 IMU sensor used to read gyroscope, accelerometer, and rotation vector data.
- **Radio Control Unit**: A microcontroller used for controller steering and speed when the tractor is under manual control and e-stop functionality .


For a detailed hardware connection overview, please refer to the [Miro Board Documentation](https://miro.com/app/board/uXjVM1yzdFo=/).

![Hardware Overview](path-to-your-screenshot.png) <!-- Replace with actual path if you add the image to the repository -->

## Program Descriptions

### BNO085_ROS_IMU_Publisher

**Purpose**: This program interfaces with the BNO085 sensor to read gyroscope, accelerometer, and rotation vector data, publishing it as a ROS `sensor_msgs/Imu` message.

- **Microcontrollers**: Teensy 3.2, Arduino Nano Every, TTGO LoRa OLED V1
- **Sensors**: Adafruit BNO085 IMU, AMS AS5048 Rotational Sensor
- **Communication Protocals Used**: USB, I2C, ROS rosserial
- **ROS Topics**: `imu/data`, std_msgs/Float32MultiArray for wheel data


**Key Features**:
1. Reads gyroscope, accelerometer, and rotation vector (quaternion) data.
2. Publishes the data to the ROS topic `imu/data`.
3. Includes magnetometer calibration status in the `orientation_covariance` field.
4. Robust initialization with multiple connection attempts to the sensor.
5. Continuous monitoring and reporting of sensor data.

*Additional programs can be added here following a similar format.*

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/jones2126/lawn_tractor_embedded_code.git
    ```

2. Navigate to the specific program directory you are interested in:
    ```bash
    cd lawn_tractor_embedded_code/BNO085_ROS_IMU_Publisher
    ```

3. Compile and upload the code using the appropriate development environment (e.g., PlatformIO, Arduino IDE).

## Usage

1. Connect the hardware components as outlined in the [Hardware Overview](#hardware-overview) section.
2. Start the ROS master node on the laptop.
3. Run the specific microcontroller program to begin communication with the ROS environment.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements, features, or bug fixes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
