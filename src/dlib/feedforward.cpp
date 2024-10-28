#include "dlib/feedforward.hpp"
#include "api.h"
#include <cmath>

namespace dlib {

// Construct a FeedForward controller
FeedForward::FeedForward(
    FFGains gains_settings) {
    
    FFGains real_gains({
        gains_settings.ka, 
        gains_settings.ks,
        gains_settings.kv
    });
    
    gains = real_gains;
}

double FeedForward::calculate(double target_velocity){
    double acceleration = 1.0;

    return(
        (std::copysign(gains.ks, target_velocity) + 
        gains.kv * target_velocity + 
        gains.ka * acceleration)
    );
}

void FeedForward::reset(){
    last_velocity = 0;
    last_time = 0;
}

}
