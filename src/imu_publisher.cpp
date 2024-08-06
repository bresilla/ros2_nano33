#include <array>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include "ros2_nano33/sensor_fusion.hpp"
#include "ros2_nano33/basic_filter.hpp"

class ImuOrientationNode : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gyro_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr acc_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mag_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ori_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub2_;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::Imu imu_msg_;
        rclcpp::Time last_time_;
        std::array<double, 3> gyro_data_;
        std::array<double, 3> acc_data_;
        std::array<double, 3> mag_data_;
        sens::SensorFusion *sens_fusion_;

    public:
        ImuOrientationNode() : Node("imu_orientation_node") {
            gyro_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/nano33/gyro", 10, std::bind(&ImuOrientationNode::gyro_callback, this, std::placeholders::_1));
            acc_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/nano33/accel", 10, std::bind(&ImuOrientationNode::acc_callback, this, std::placeholders::_1));
            mag_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/nano33/mag", 10, std::bind(&ImuOrientationNode::mag_callback, this, std::placeholders::_1));
            ori_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/nano33/ori", 10, std::bind(&ImuOrientationNode::ori_callback, this, std::placeholders::_1));
            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/nano33/imu", 10);
            imu_pub2_ = this->create_publisher<sensor_msgs::msg::Imu>("/nano33/imu_hardware", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ImuOrientationNode::timer_callback, this));
            last_time_ = this->now();
            // sens_fusion_ = new sens::ExtendedKalmanFilter();
            sens_fusion_ = new sens::DataFusion();
        }

    private:
        void timer_callback() {
            if (gyro_data_.empty() || acc_data_.empty()) return;
            auto current_time = this->now();
            auto delta_time = current_time - last_time_;
            double dt = delta_time.seconds();
            last_time_ = current_time;
            // Update orientation with the actual delta time
            sens_fusion_->update(gyro_data_, acc_data_, dt);
            // Create IMU message
            imu_msg_.angular_velocity.x = gyro_data_[0];
            imu_msg_.angular_velocity.y = gyro_data_[1];
            imu_msg_.angular_velocity.z = gyro_data_[2];

            imu_msg_.linear_acceleration.x = acc_data_[0];
            imu_msg_.linear_acceleration.y = acc_data_[1];
            imu_msg_.linear_acceleration.z = acc_data_[2];

            imu_msg_.orientation.x = sens_fusion_->get_state().x;
            imu_msg_.orientation.y = sens_fusion_->get_state().y;
            imu_msg_.orientation.z = sens_fusion_->get_state().z;
            imu_msg_.orientation.w = sens_fusion_->get_state().w;

            imu_msg_.header.stamp = this->now();
            imu_msg_.header.frame_id = "imu_frame";
            imu_pub_->publish(imu_msg_);
            RCLCPP_INFO(this->get_logger(), "Published IMU data");
        }

        void gyro_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            gyro_data_[0] = msg->data[0];
            gyro_data_[1] = msg->data[1];
            gyro_data_[2] = msg->data[2];
        }

        void acc_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            acc_data_[0] = msg->data[0];
            acc_data_[1] = msg->data[1];
            acc_data_[2] = msg->data[2];
        }

        void mag_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            mag_data_[0] = msg->data[0];
            mag_data_[1] = msg->data[1];
            mag_data_[2] = msg->data[2];
        }

        void ori_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (gyro_data_.empty() || acc_data_.empty()) return;
            tf2::Quaternion q;
            q.setRPY(msg->data[0], msg->data[1], msg->data[2]);
            sensor_msgs::msg::Imu imu_msg2_ = imu_msg_;
            imu_msg2_.orientation.x = q.x();
            imu_msg2_.orientation.y = q.y();
            imu_msg2_.orientation.z = q.z();
            imu_msg2_.orientation.w = q.w();
            imu_msg2_.header.stamp = this->now();
            imu_msg2_.header.frame_id = "imu_frame";
            imu_pub2_->publish(imu_msg2_);
            RCLCPP_INFO(this->get_logger(), "Published IMU data");
        }

   };

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOrientationNode>());
    rclcpp::shutdown();
    return 0;
}
