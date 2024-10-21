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

    dlib::IMU imu = dlib::IMU(
        4
    );

    // Create a new PID for whatever you need!
    dlib::PID drive_pid = dlib::PID(
        // Adjust each gain for a more accurate PID.
        {0,0,0},
        1
    );

    dlib::PID turn_pid = dlib::PID(
        {0,0,0},
        1
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

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Try some movements!
    dlib::move_voltage(robot,127);
    pros::delay(500);
    dlib::brake_motors(robot);
}

void opcontrol() {
    while(true){
        // Update position struct
        position = dlib::get_position(robot, false);

        // basic arcade using dLib
        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        dlib::arcade(robot, power, turn);

        // Update position on UI
        pros::lcd::print(1, "X: %lf", position.x);
        pros::lcd::print(2, "Y: %lf", position.y);
        pros::lcd::print(3, "Theta: %lf", position.theta);
        

        pros::delay(20);
    }
}