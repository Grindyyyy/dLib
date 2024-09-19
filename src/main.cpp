#include "main.h"
#include "dlib/dlib.hpp"
#include "pros/misc.h"

// Robot Constructor
// Initialize your robot inside of this constructor!
struct Robot {
    dlib::Chassis chassis = dlib::Chassis(
		{1,2,3},
    	{1,2,3},
    	1,
    	1
    );

    dlib::PID drive_pid = dlib::PID(
        {},
        1
    );

    dlib::PID turn_pid = dlib::PID(
        {},
        1
    );

    dlib::Odom odom = dlib::Odom(
        1,
        1
    );

    dlib::Chassis& get_chassis() {
        return chassis;
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

// instantiate a Controller object
pros::Controller master(pros::E_CONTROLLER_MASTER);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Try some movements!
    dlib::move_voltage(robot,127);
}

void opcontrol() {
    while(true){
        // basic arcade using dLib
        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        dlib::arcade(robot, power, turn);

        pros::delay(20);
    }
}