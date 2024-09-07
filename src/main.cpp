#include "main.h"
#include "dlib/dlib.hpp"

struct Robot {
    dlib::Chassis chassis = dlib::Chassis(
		nullptr,
    	nullptr,
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

void initialize() {
	
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
}



void opcontrol() {

}