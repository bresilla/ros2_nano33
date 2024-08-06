#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <thread>
#include <sstream>
#include <iostream>

class SensorReaderNode : public rclcpp::Node {
    private:
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gyro_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mag_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr acc_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ori_pub_;
        int serial_fd_;
        std::thread reader_thread_;
        float gate_g = 5.0;
        float gate_a = 0.1;
        float gate_m = 0.1;

    public:
        SensorReaderNode() : Node("sensor_reader_node") {
            gyro_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/nano33/gyro", 10);
            mag_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/nano33/mag", 10);
            acc_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/nano33/accel", 10);
            ori_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/nano33/ori", 10);

            std::string port = "/dev/nano33ble";  // Update with your serial port
            if (!setupSerial(port, B115200)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
                rclcpp::shutdown();
            } else {
                reader_thread_ = std::thread(&SensorReaderNode::readSerialData, this);
            }
        }

        ~SensorReaderNode() {
            if (reader_thread_.joinable()) {
                reader_thread_.join();
            }
            close(serial_fd_);
        }

    private:
        bool setupSerial(const std::string &port, speed_t baud) {
            serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
            if (serial_fd_ < 0) {
                std::cerr << "Error opening " << port << ": " << strerror(errno) << std::endl;
                return false;
            }
            termios tty;
            if (tcgetattr(serial_fd_, &tty) != 0) {
                std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
                return false;
            }
            cfsetospeed(&tty, baud);
            cfsetispeed(&tty, baud);
            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
            tty.c_iflag &= ~IGNBRK;                         // disable break processing
            tty.c_lflag = 0;                                // no signaling chars, no echo, no canonical processing
            tty.c_oflag = 0;                                // no remapping, no delays
            tty.c_cc[VMIN]  = 1;                            // read doesn't block
            tty.c_cc[VTIME] = 1;                            // 0.1 seconds read timeout
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
            tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
            tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
            tty.c_cflag |= 0;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;
            if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
                std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
                return false;
            }
            return true;
        }

        void readSerialData(){
            char buf[256];
            std::string line;
            while (rclcpp::ok()) {
                int n = read(serial_fd_, buf, sizeof(buf) - 1);
                if (n > 0) {
                    buf[n] = '\0';
                    line += std::string(buf, n);
                    if (line.back() == '\n') {
                        processLine(line);
                        line.clear();
                    }
                }
            }
        }

        void processLine(const std::string &line) {
            std::istringstream iss(line);
            std::string sensor;
            iss >> sensor;

            std_msgs::msg::Float32MultiArray msg;
            std::vector<float> data(3);

            if (sensor == "Gyroscope:") {
                iss >> data[0] >> data[1] >> data[2];
                if (iss.fail()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse gyroscope data: '%s'", line.c_str());
                } else {
                    if (abs(data[0]) < gate_g) data[0] = 0;
                    if (abs(data[1]) < gate_g) data[1] = 0;
                    if (abs(data[2]) < gate_g) data[2] = 0;
                    msg.data = data;
                    gyro_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Gyro: %.2f %.2f %.2f", data[0], data[1], data[2]);
                }
            } else if (sensor == "Magnetometer:") {
                iss >> data[0] >> data[1] >> data[2];
                if (iss.fail()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse magnetometer data: '%s'", line.c_str());
                } else {
                    if (abs(data[0]) < gate_m) data[0] = 0;
                    if (abs(data[1]) < gate_m) data[1] = 0;
                    if (abs(data[2]) < gate_m) data[2] = 0;
                    msg.data = data;
                    mag_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Mag: %.2f %.2f %.2f", data[0], data[1], data[2]);
                }
            } else if (sensor == "Acceleration:") {
                iss >> data[0] >> data[1] >> data[2];
                if (iss.fail()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse acceleration data: '%s'", line.c_str());
                } else {
                    if (abs(data[0]) < gate_a) data[0] = 0;
                    if (abs(data[1]) < gate_a) data[1] = 0;
                    if (abs(data[2]) < gate_a) data[2] = 0;
                    msg.data = data;
                    acc_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "Acc: %.2f %.2f %.2f", data[0], data[1], data[2]);
                }
            } else if (sensor == "Orientation:") {
                iss >> data[0] >> data[1] >> data[2];
                if (iss.fail()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse orientation data: '%s'", line.c_str());
                } else {
                    msg.data = data;
                    ori_pub_->publish(msg);
                    // RCLCPP_INFO(this->get_logger(), "Ori: %.2f %.2f %.2f", data[0], data[1], data[2]);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown sensor type: '%s'", line.c_str());
            }
        }

    };

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorReaderNode>());
    rclcpp::shutdown();
    return 0;
}
