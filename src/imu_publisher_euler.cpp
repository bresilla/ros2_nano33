#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>

class ImuPublisherNode : public rclcpp::Node {
public:
    ImuPublisherNode() : Node("imu_publisher_node") {
        gyro_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "gyro", 10, std::bind(&ImuPublisherNode::gyroCallback, this, std::placeholders::_1));

        acc_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "acc", 10, std::bind(&ImuPublisherNode::accCallback, this, std::placeholders::_1));

        ori_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "ori", 10, std::bind(&ImuPublisherNode::oriCallback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    }

private:
    void gyroCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_msg_.angular_velocity.x = msg->data[0];
        imu_msg_.angular_velocity.y = msg->data[1];
        imu_msg_.angular_velocity.z = msg->data[2];
        publishImu();
    }

    void accCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_msg_.linear_acceleration.x = msg->data[0];
        imu_msg_.linear_acceleration.y = msg->data[1];
        imu_msg_.linear_acceleration.z = msg->data[2];
        publishImu();
    }

    void publishImu() {
        imu_msg_.header.stamp = this->now();
        imu_msg_.header.frame_id = "imu_frame";
        imu_pub_->publish(imu_msg_);
        RCLCPP_INFO(this->get_logger(), "Published IMU data");
    }

    void oriCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        tf2::Quaternion q;
        q.setRPY(msg->data[0], msg->data[1], msg->data[2]);
        imu_msg_.orientation.x = q.x();
        imu_msg_.orientation.y = q.y();
        imu_msg_.orientation.z = q.z();
        imu_msg_.orientation.w = q.w();
        publishImu();
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gyro_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr acc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ori_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    sensor_msgs::msg::Imu imu_msg_;
    std::mutex mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
