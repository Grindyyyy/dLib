#pragma once
#include <iostream>

namespace dlib {

class TrapMotionProfile {
    private:

        double maxAccel;
        double maxVelo;
        double totalDistance;

        double totalTime;
        
        double accel_time = maxVelo/maxAccel;
        double accel_distance = (0.5 * (maxAccel*(accel_time*accel_time)));

        double coast_distance = totalDistance - (0.5*maxAccel*(accel_time*accel_time));
        double coast_time = coast_distance / maxVelo;

    public:
        // pass Max accel & Max velo for constructor
        TrapMotionProfile(double maxAcceleration, double maxVelocity);

        // trap Velocity profile
        double velocity_at(double curTime);
        // should return position eventually

};
}