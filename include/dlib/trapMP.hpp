#pragma once
#include <iostream>

namespace dlib {

class TrapMotionProfile {
    private:

        double maxAccel;
        double maxVelo;
        double totalDistance;

        double totalTime;

    public:
        // pass Max accel & Max velo for constructor
        TrapMotionProfile(double maxAcceleration, double maxVelocity, double totalDistance);

        // trap Velocity profile
        double velocity_at(double curTime);
        // should return position eventually

};
}