#pragma once

#include <array>
#include <chrono>
#include <math.h>

namespace sens {
    struct Quaternion {
        double w, x, y, z;
        Quaternion(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0) : w(w), x(x), y(y), z(z) {};
        Quaternion operator*(const Quaternion &q) const {
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,
                w*q.x + x*q.w + y*q.z - z*q.y,
                w*q.y - x*q.z + y*q.w + z*q.x,
                w*q.z + x*q.y - y*q.x + z*q.w
            );
        };
        void normalize(){
            double norm = std::sqrt(w * w + x * x + y * y + z * z);
            if (norm > 1e-6) {  // Avoid division by zero
                w /= norm;
                x /= norm;
                y /= norm;
                z /= norm;
            } else {
                w = 1.0; x = 0.0; y = 0.0; z = 0.0;  // Reset to default
            }
        };
    };

    class SensorFusion {
        public:
            virtual void update(const std::array<double, 3> &gyro, const std::array<double, 3> &accel, double d_time) = 0;
            virtual Quaternion get_state() const = 0;
    };
}
