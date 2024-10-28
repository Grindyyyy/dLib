#pragma once
#include <iostream>
#include <cmath>

namespace dlib {

class TrapMotionProfile {
    private:

        double maxAccel;
        double maxVelo;
        double totalDistance;

        double totalTime;
        
        double accel_time;
        double accel_distance;

        double coast_distance;
        double coast_time;

    public:
        // pass Max accel & Max velo for constructor
        TrapMotionProfile(double maxAcceleration, double maxVelocity, double totalDistance);

        // trap Velocity profile
        double velocity_at(double curTime);
        // should return position eventually

};
}