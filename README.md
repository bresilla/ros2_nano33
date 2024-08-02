# ros2_nano33

## Overview

The `ros2_nano33` package reads data from a serial port connected to a sensor module (e.g., gyroscope, magnetometer, accelerometer) on the Arduino Nano 33 BLE Sense board and publishes the data to different ROS 2 topics. It also includes a node that subscribes to these topics and publishes the data as a `sensor_msgs/msg/Imu` message.

## Features

- Reads data from a serial port.
- Parses sensor data (gyroscope, magnetometer, accelerometer).
- Publishes parsed data to corresponding ROS 2 topics.
- Subscribes to sensor data topics and publishes `sensor_msgs/msg/Imu` messages.

## Topics

### Published Topics

- `/gyro` (`std_msgs/msg/Float32MultiArray`): Gyroscope data.
- `/mag` (`std_msgs/msg/Float32MultiArray`): Magnetometer data.
- `/acc` (`std_msgs/msg/Float32MultiArray`): Accelerometer data.
- `/imu` (`sensor_msgs/msg/Imu`): Combined IMU data.

### Subscribed Topics

- `/gyro` (`std_msgs/msg/Float32MultiArray`): Gyroscope data.
- `/mag` (`std_msgs/msg/Float32MultiArray`): Magnetometer data.
- `/acc` (`std_msgs/msg/Float32MultiArray`): Accelerometer data.

## Installation

1. **Install ROS 2**: Follow the instructions from the [official ROS 2 documentation](https://docs.ros.org/en/humble/Installation.html) to install ROS 2.

2. **Install dependencies**:
   ```sh
   sudo apt-get update
   sudo apt-get install libserial-dev
   ```

3. **Create a ROS 2 workspace** (if you don't have one already):
   ```sh
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

4. **Clone the package** into your workspace:
   ```sh
   cd ~/ros2_ws/src
   git clone <repository_url> ros2_nano33
   ```

5. **Build the package**:
   ```sh
   cd ~/ros2_ws
   colcon build --packages-select ros2_nano33
   ```

6. **Source the workspace**:
   ```sh
   source install/setup.bash
   ```

## Usage

1. **Run the sensor reader node**:
   ```sh
   ros2 run ros2_nano33 sensor_reader_node
   ```

2. **Run the IMU publisher node**:
   ```sh
   ros2 run ros2_nano33 imu_publisher_node
   ```

## Configuration

Update the serial port and baud rate in `sensor_reader_node.cpp` if needed:
```cpp
std::string port = "/dev/ttyUSB0";  // Update with your serial port
unsigned long baud = 9600;  // Update with your baud rate
```

## Example Output

The `sensor_reader_node` reads lines like:
```
Gyroscope:      0.43    -0.06   -0.06
Magnetometer:   -74.00  0.00    58.00
Acceleration:   0.16    -0.10   0.99
```
and publishes the data to the corresponding topics.

The `imu_publisher_node` subscribes to the `/gyro`, `/mag`, and `/acc` topics, aggregates the sensor data, and publishes it as a `sensor_msgs/msg/Imu` message.


## Make so the sensor can be at specific path


### 1. Identify the Device Attributes

First, connect your Arduino Nano 33 BLE to your computer and run the following command to list its attributes:

```bash
udevadm info --name=/dev/ttyACM0 --attribute-walk
```

Look for the attributes of the device, such as `idVendor`, `idProduct`, and `product`. For the Arduino Nano 33 BLE, these typically are:

```plaintext
ATTRS{idVendor}=="2341"
ATTRS{idProduct}=="805a"
ATTRS{product}=="Nano 33 BLE"
```

### 2. Create a Udev Rules File

Create a new udev rules file using the command:

```bash
sudo nano /etc/udev/rules.d/99-arduino-nano-33-ble.rules
```

### 3. Add the Udev Rule

Add the following line to the file to create a symbolic link `/dev/nano33ble`:

```plaintext
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="805a", ATTRS{product}=="Nano 33 BLE", SYMLINK+="nano33ble"
```

Save the file and exit the editor (`Ctrl+X`, then `Y`, and `Enter`).

### 4. Reload Udev Rules

Reload the udev rules to apply the changes:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 5. Verify the Symbolic Link

Unplug and reconnect your Arduino Nano 33 BLE, then verify that the symbolic link has been created:

```bash
ls -l /dev/nano33ble
```

You should see output indicating that the symbolic link points to the correct device file (e.g., `ttyACM0`).


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## Acknowledgments

- Thanks to the [ROS 2 community](https://discourse.ros.org/) for their support.
- This package was inspired by the [Arduino Nano 33 BLE Sense](https://store.arduino.cc/usa/nano-33-ble-sense) board.
- [Libserial](https://github.com/crayzeewulf/libserial) is used to read data from the serial port.
