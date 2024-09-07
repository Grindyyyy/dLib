#include "api.h"

namespace dlib {
struct Chassis {
    std::unique_ptr<pros::MotorGroup> left;
    std::unique_ptr<pros::MotorGroup> right;
    double wheel_diameter;
    double gear_ratio;
};
}