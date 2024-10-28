#include "dlib/trapMP.hpp"
#include "api.h"
#include <cmath>

namespace dlib {

// Construct a TrapMotionProfile controller (should be in inches)
TrapMotionProfile::TrapMotionProfile(double maxAcceleration, double maxVelocity, double totalDistance) {
    maxAccel = maxAcceleration;
    maxVelo = maxVelocity;

    // get the acceleration time
    accel_time = maxVelo/maxAccel;

    // set decel time to accelTime as they are the same
    double decelTime = accel_time;

    // get the acceleration distance via x = 1/2at^2
    accel_distance = (maxAcceleration * std::pow(accel_time, 2)) / 2;

    // sets the deceleration distance to accelDistance as they should be the same
    double decelDistance = accel_distance;

    // gets coast distance by totalx - (accelx + decelx)
    coast_distance = totalDistance - accel_distance - decelDistance;

    // if the coast distance is less than zero, find out max accel time
    if (coast_distance < 0) {
        
        // find the accel time via the equation sqrt(2x/a) = t
        accel_time = std::sqrt(totalDistance/maxAcceleration);

        // decel time will be the same as accel time
        decelTime = accel_time;

        // no time for coasting if the coast distance is less than zero
        coast_distance = 0;
    } 

    // sets coast_time by calculating the coastx/maxVelo
    coast_time = coast_distance/maxVelocity;

    // total time is just all sectors added together
    totalTime = accel_time + decelTime + coast_time;
}

Setpoint TrapMotionProfile::velocity_at(double curTime){
    // if before coasting
    if(curTime <= accel_time){
        return Setpoint(setpoint.position,maxAccel * curTime); 
    }
    // if during coasting
    else if(curTime <= accel_time + coast_time){
        return Setpoint(setpoint.position,maxVelo);
    }
    // if after coasting
    else if(curTime <= accel_time*2 + coast_time){
        return Setpoint(setpoint.position,maxAccel * (totalTime - curTime));
    }
    else{
        return Setpoint(setpoint.position,0);
    }
}

}
