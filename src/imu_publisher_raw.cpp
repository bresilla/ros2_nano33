#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chrono>
#include <mutex>

class IMUOrientation {
public:
    struct Quaternion {
        double w, x, y, z;
        Quaternion operator*(const Quaternion &q) const {
            return {
                w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w
            };
        }

        void normalize() {
            double norm = std::sqrt(w * w + x * x + y * y + z * z);
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        }
    };

    IMUOrientation() {
        orientation = {1.0, 0.0, 0.0, 0.0};
    }

    void update(const std::array<double, 3> &gyro, const std::array<double, 3> &acc, double dt) {
        orientation = integrateGyro(gyro, orientation, dt);
        orientation.normalize();
        // Use accelerometer data to correct orientation drift here if necessary
        // This can be done using complementary filter or other sensor fusion techniques
    }

    Quaternion get_quaternion() const {
        return orientation;
    }

private:
    Quaternion orientation;

    Quaternion integrateGyro(const std::array<double, 3> &gyro, const Quaternion &q, double dt) const {
        // Convert gyroscope data (degrees/sec) to radians/sec
        double gyro_rad[3] = {gyro[0] * M_PI / 180.0, gyro[1] * M_PI / 180.0, gyro[2] * M_PI / 180.0};
        // Calculate the norm of the gyroscope vector
        double omega = std::sqrt(gyro_rad[0] * gyro_rad[0] + gyro_rad[1] * gyro_rad[1] + gyro_rad[2] * gyro_rad[2]);
        Quaternion delta_q;
        if (omega > 0) {
            double axis[3] = {gyro_rad[0] / omega, gyro_rad[1] / omega, gyro_rad[2] / omega};
            double angle = omega * dt;
            delta_q = {
                std::cos(angle / 2.0),
                axis[0] * std::sin(angle / 2.0),
                axis[1] * std::sin(angle / 2.0),
                axis[2] * std::sin(angle / 2.0)
            };
        } else {
            delta_q = {1.0, 0.0, 0.0, 0.0};
        }

        Quaternion new_q = q * delta_q;
        return new_q;
    }
};

class ImuOrientationNode : public rclcpp::Node {
private:
    message_filters::Subscriber<std_msgs::msg::Float32MultiArray> gyro_sub_;
    message_filters::Subscriber<std_msgs::msg::Float32MultiArray> acc_sub_;
    typedef message_filters::sync_policies::ApproximateTime<std_msgs::msg::Float32MultiArray, std_msgs::msg::Float32MultiArray> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    IMUOrientation imu_orientation;
    std::mutex mutex_;
    sensor_msgs::msg::Imu imu_msg_;
    rclcpp::Time last_time_;

public:
    ImuOrientationNode()
        : Node("imu_orientation_node"),
          gyro_sub_(this, "gyro", 10),
          acc_sub_(this, "acc", 10),
          sync_(MySyncPolicy(10), gyro_sub_, acc_sub_) {
        sync_.registerCallback(std::bind(&ImuOrientationNode::callback, this, std::placeholders::_1, std::placeholders::_2));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        last_time_ = this->now();
    }

private:
    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr gyro_msg, const std_msgs::msg::Float32MultiArray::SharedPtr acc_msg) {
        RCLCPP_INFO(this->get_logger(), "Received gyro data: %f, %f, %f", gyro_msg->data[0], gyro_msg->data[1], gyro_msg->data[2]);
        RCLCPP_INFO(this->get_logger(), "Received acc data: %f, %f, %f", acc_msg->data[0], acc_msg->data[1], acc_msg->data[2]);

        std::array<double, 3> gyro_data = {gyro_msg->data[0], gyro_msg->data[1], gyro_msg->data[2]};
        std::array<double, 3> acc_data = {acc_msg->data[0], acc_msg->data[1], acc_msg->data[2]};

        auto current_time = this->now();
        auto delta_time = current_time - last_time_;
        double dt = delta_time.seconds();
        last_time_ = current_time;

        // Update orientation with gyro and accelerometer data
        imu_orientation.update(gyro_data, acc_data, dt);

        std::lock_guard<std::mutex> lock(mutex_);
        // Create IMU message
        imu_msg_.angular_velocity.x = gyro_data[0];
        imu_msg_.angular_velocity.y = gyro_data[1];
        imu_msg_.angular_velocity.z = gyro_data[2];
        imu_msg_.linear_acceleration.x = acc_data[0];
        imu_msg_.linear_acceleration.y = acc_data[1];
        imu_msg_.linear_acceleration.z = acc_data[2];

        auto quaternion = imu_orientation.get_quaternion();
        imu_msg_.orientation.x = quaternion.x;
        imu_msg_.orientation.y = quaternion.y;
        imu_msg_.orientation.z = quaternion.z;
        imu_msg_.orientation.w = quaternion.w;

        // Publish IMU message
        publish_imu();
    }

    void publish_imu() {
        imu_msg_.header.stamp = this->now();
        imu_msg_.header.frame_id = "imu_frame";
        imu_pub_->publish(imu_msg_);
        RCLCPP_INFO(this->get_logger(), "Published IMU data");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOrientationNode>());
    rclcpp::shutdown();
    return 0;
}
