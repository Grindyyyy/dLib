#include "dlib/trapMP.hpp"
#include "api.h"
#include <cmath>

namespace dlib {

// Construct a TrapMotionProfile controller (should be in inches)
TrapMotionProfile::TrapMotionProfile(double maxAcceleration, double maxVelocity, double totalDistance) {
    maxAccel = maxAcceleration;
    maxVelo = maxVelocity;

    // figures out the accel time using Vmax/Amax
    double accelTime = maxVelocity/maxAcceleration;

    // derived from the 1/2at^2 constant accel equation from physics
    double accelDistance = 0.5*maxAcceleration*(accelTime*accelTime);

    // determine if there can be a coasting section for the TrapMP
    if ((accelDistance*2) > totalDistance) {
        // no coast sector, so it has to figure out the max speed it can do

        // calculate the max distance it can accelerate or decelerate
        double accelMaxDistance = totalDistance/2.0;

        // based on the equation sqrt(2x/a) = t derived from x = 1/2at^2
        double accelMaxTime = std::sqrt(accelMaxDistance*2/maxAcceleration);

        // sets the total time to accelMaxTime*2 due to both portions.
        totalTime = accelMaxTime*2;


    } else {
        // Coast sector exists, calculate time with coast

        // simplified would be (total_distance - accel portions)/maxVelocity
        double coastSectorTime = (totalDistance-(accelDistance*2))/maxVelocity;

        // gives the total predicted time for the movement
        totalTime = accelTime*2 + coastSectorTime;
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
