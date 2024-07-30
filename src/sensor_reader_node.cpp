#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <thread>

class SensorReaderNode : public rclcpp::Node
{
public:
    SensorReaderNode() : Node("sensor_reader_node")
    {
        gyro_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("gyro", 10);
        mag_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("mag", 10);
        acc_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("acc", 10);

        std::string port = "/dev/ttyUSB0";  // Update with your serial port
        unsigned long baud = 9600;  // Update with your baud rate
        serial_.setPort(port);
        serial_.setBaudrate(baud);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);
        serial_.open();

        if (!serial_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
            rclcpp::shutdown();
        } else {
            reader_thread_ = std::thread(&SensorReaderNode::readSerialData, this);
        }
    }

    ~SensorReaderNode()
    {
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
        serial_.close();
    }

private:
    void readSerialData()
    {
        while (rclcpp::ok()) {
            if (serial_.available()) {
                std::string line = serial_.readline(1024, "\n");
                std::istringstream iss(line);
                std::string sensor;
                iss >> sensor;

                std_msgs::msg::Float32MultiArray msg;
                std::vector<float> data(3);

                if (sensor == "Gyroscope:") {
                    iss >> data[0] >> data[1] >> data[2];
                    msg.data = data;
                    gyro_pub_->publish(msg);
                } else if (sensor == "Magnetometer:") {
                    iss >> data[0] >> data[1] >> data[2];
                    msg.data = data;
                    mag_pub_->publish(msg);
                } else if (sensor == "Acceleration:") {
                    iss >> data[0] >> data[1] >> data[2];
                    msg.data = data;
                    acc_pub_->publish(msg);
                }
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gyro_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr acc_pub_;
    serial::Serial serial_;
    std::thread reader_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorReaderNode>());
    rclcpp::shutdown();
    return 0;
}
