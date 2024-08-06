#pragma once

#include <array>
#include "ros2_nano33/sensor_fusion.hpp"

namespace sens {
    class DataFusion : public SensorFusion {
    public:
        Quaternion quaterion;
        DataFusion() : quaterion({1.0, 0.0, 0.0, 0.0}) {}

        void update(const std::array<double, 3> &gyro, const std::array<double, 3> &accel, double dt) {
            // Integrate gyroscope data
            Quaternion gyro_update = update_gyro(gyro, quaterion, dt);
            gyro_update.normalize();

            // Convert accelerometer data to quaternion
            Quaternion accel_quat = update_accel(accel);

            // Apply complementary filter to combine accelerometer and gyroscope data
            quaterion = slerp(gyro_update, accel_quat, 1.0);
            quaterion.normalize();
        }

        Quaternion get_state() const {
            return quaterion;
        }

    private:
        Quaternion update_gyro(const std::array<double, 3> &gyro, const Quaternion &q, double dt) const {
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

        Quaternion update_accel(const std::array<double, 3> &accel) const {
            double norm = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
            if (norm == 0) return {1.0, 0.0, 0.0, 0.0}; // Avoid division by zero
            double x = accel[0] / norm;
            double y = accel[1] / norm;
            double z = accel[2] / norm;
            // Compute the pitch and roll from accelerometer data
            double pitch = std::atan2(y, std::sqrt(x * x + z * z));
            double roll = std::atan2(-x, z);
            // Convert pitch and roll to quaternion
            double cy = std::cos(pitch * 0.5);
            double sy = std::sin(pitch * 0.5);
            double cr = std::cos(roll * 0.5);
            double sr = std::sin(roll * 0.5);
            Quaternion q = {
                cr * cy,
                sr * cy,
                cr * sy,
                -sr * sy
            };
            return q;
        }

        Quaternion slerp(const Quaternion &q1, const Quaternion &q2, double t) const {
            // Compute the cosine of the angle between the two vectors.
            double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
            // If the dot product is negative, slerp won't take the shorter path.
            if (dot < 0.0f) {
                dot = -dot;
            }
            const double threshold = 0.9995;
            if (dot > threshold) {
                // If the inputs are too close for comfort, linearly interpolate
                // and normalize the result.
                Quaternion result = {
                    q1.w + t * (q2.w - q1.w),
                    q1.x + t * (q2.x - q1.x),
                    q1.y + t * (q2.y - q1.y),
                    q1.z + t * (q2.z - q1.z)
                };
                result.normalize();
                return result;
            }
            // Since dot is in range [0, threshold], acos is safe
            double theta_0 = std::acos(dot);  // theta_0 = angle between input vectors
            double theta = theta_0 * t;       // theta = angle between v0 and result
            double sin_theta = std::sin(theta);     // compute this value only once
            double sin_theta_0 = std::sin(theta_0); // compute this value only once
            double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
            double s1 = sin_theta / sin_theta_0;
            return {
                (q1.w * s0 + q2.w * s1),
                (q1.x * s0 + q2.x * s1),
                (q1.y * s0 + q2.y * s1),
                (q1.z * s0 + q2.z * s1)
            };
        }
    };
}
