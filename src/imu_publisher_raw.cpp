#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>


class IMUOrientation {
    public:
        struct Quaternion {
            double w, x, y, z;
            Quaternion operator*(const Quaternion &q) const {
                return {
                    w * q.w - x * q.x - y * q.y - z * q.z,
                    w * q.x + x * q.w + y * q.z - z * q.y,
                    w * q.y - x * q.z + y * q.w + z * q.x,
                    w * q.z + x * q.y - y * q.x + z * q.w};
            }

            void normalize(){
                double norm = std::sqrt(w * w + x * x + y * y + z * z);
                w /= norm;
                x /= norm;
                y /= norm;
                z /= norm;
            }
        };

        IMUOrientation(){
            // Initialize the quaternion representing the initial orientation
            orientation = {1.0, 0.0, 0.0, 0.0};
        }

        void updateWithGyroData(const std::array<double, 3> &gyro, double dt){
            orientation = integrateGyro(gyro, orientation, dt);
            orientation.normalize();
        }

        Quaternion getCurrentQuaternion() const{
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
                double angle = omega * dt; // We use the actual delta time here
                delta_q = {
                    std::cos(angle / 2.0),
                    axis[0] * std::sin(angle / 2.0),
                    axis[1] * std::sin(angle / 2.0),
                    axis[2] * std::sin(angle / 2.0)};
            } else {
                delta_q = {1.0, 0.0, 0.0, 0.0};
            }

            Quaternion new_q = q * delta_q;
            return new_q;
        }
};

class ImuOrientationNode : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        IMUOrientation imu_orientation;
        rclcpp::Time last_time_;

    public:
        ImuOrientationNode() : Node("imu_orientation_node") {
            subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("gyro", 10, std::bind(&ImuOrientationNode::acc_callback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
            last_time_ = this->now();
        }

    private:
        void acc_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() != 3) {
                RCLCPP_WARN(this->get_logger(), "Received data does not have exactly 3 elements.");
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "Received data: %f, %f, %f", msg->data[0], msg->data[1], msg->data[2]);
            }
            std::array<double, 3> gyro_data = {msg->data[0], msg->data[1], msg->data[2]};
            auto current_time = this->now();
            auto delta_time = current_time - last_time_;
            double dt = delta_time.seconds();
            last_time_ = current_time;
            // Update orientation with the actual delta time
            imu_orientation.updateWithGyroData(gyro_data, dt);
            // Create IMU message
            auto imu_msg = sensor_msgs::msg::Imu();
            auto quaternion = imu_orientation.getCurrentQuaternion();
            imu_msg.orientation.x = quaternion.x;
            imu_msg.orientation.y = quaternion.y;
            imu_msg.orientation.z = quaternion.z;
            imu_msg.orientation.w = quaternion.w;
            // Publish IMU message
            publisher_->publish(imu_msg);
        }
   };

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOrientationNode>());
    rclcpp::shutdown();
    return 0;
}
