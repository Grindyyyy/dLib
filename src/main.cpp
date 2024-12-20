#include "main.h"
#include "dlib/dlib.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
#include <cstdio>

// ** CLANGD IS RECOMMENDED ** //

// Robot Constructor
// Initialize your robot inside of this constructor!
struct Robot {
    dlib::Chassis chassis = dlib::Chassis(
		{-14,-4,-15},
    	{18,12,13},
    	3.25,
    	450
    );

    dlib::IMU imu = dlib::IMU(
        17,
        // IMU return scalar
        1.01107975
    );

    // Create a new PID for whatever you need!
    dlib::PID drive_pid = dlib::PID(
        // Adjust each gain for a more accurate PID.
        // kp, ki, kd
        {0,0,0},
        1
    );

    dlib::PID turn_pid = dlib::PID(
        // kp, ki, kd
        {0,0,0},
        1
    );

    dlib::FeedForward drive_feed_forward = dlib::FeedForward(
        {0,0,0}
    );

    dlib::FeedForward turn_feed_forward = dlib::FeedForward(
        {0,0,0}
    );

    dlib::Odom odom = dlib::Odom();

    dlib::Chassis& get_chassis() {
        return chassis;
    }

    dlib::IMU& get_imu() {
        return imu;
    } 

    dlib::PID& get_drive_pid() {
        return drive_pid;
    }

    dlib::PID& get_turn_pid() {
        return turn_pid;
    } 

    dlib::Odom& get_odom() {
        return odom;
    }
};

// instantiate a Robot object
Robot robot = Robot();

// instantiate a Position struct
dlib::Position position = dlib::get_position(robot, false);

// instantiate a Controller object
pros::Controller master(pros::E_CONTROLLER_MASTER);

void initialize() {
    dlib::calibrate(robot);
    pros::lcd::initialize();
    dlib::start_odom_update_loop(robot);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
 // try some movements!
}

void opcontrol() {
    while(true){
        // Update position struct
        position = dlib::get_position(robot, false);

        // basic arcade using dLib
        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        dlib::arcade(robot, power, turn);
        
        pros::lcd::set_text(0, "x: " + std::to_string(position.x));
        pros::lcd::set_text(1, "y: " + std::to_string(position.y));
        pros::lcd::set_text(2, "theta: " + std::to_string(position.theta));

        pros::delay(20);
    }
}