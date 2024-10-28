#include "dlib/trapMP.hpp"
#include "api.h"
#include <cmath>

namespace dlib {

// Construct a TrapMotionProfile controller (should be in inches)
TrapMotionProfile::TrapMotionProfile(double maxAcceleration, double maxVelocity, double distance) {
    maxAccel = maxAcceleration;
    maxVelo = maxVelocity;

    double accelTime = maxVelocity/maxAcceleration;

    // derived from the 1/2at^2 constant accel equation from physics
    double accelDistance = 0.5*maxAcceleration*(accelTime*accelTime);

    // determine 
    if ((accelDistance*2) > distance) {
        // no coast sector
    }



}

double TrapMotionProfile::velocity_at(double curTime){
    double accel_time = maxVelo/maxAccel;

    double coast_distance = totalDistance - (0.5*maxAccel*(accel_time*accel_time));
    double coast_time = coast_distance / maxVelo;
    if(curTime >= 0 && curTime <= accel_time){
        return maxAccel * curTime;
    }
    if(curTime >= accel_time && curTime <= accel_time + coast_time){
        return maxVelo;
    }
    if(curTime >= accel_time + coast_time && curTime <= accel_time*2 + coast_time){
        return maxAccel * (totalTime - curTime);
    }

    
}

}
