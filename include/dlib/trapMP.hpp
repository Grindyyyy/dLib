#pragma once
#include <iostream>

namespace dlib {

class TrapMotionProfile {
    private:

        double maxAccel;
        double maxVelo;

        double totalTime;

    public:
        // pass Max accel & Max velo for constructor
        TrapMotionProfile(double maxAcceleration, double maxVelocity, double distance);

        // trap Velocity profile
        double velocity_at(double curTime);
        // should return position eventually

};
}