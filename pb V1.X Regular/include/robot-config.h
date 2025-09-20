#pragma once

#include "main.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "robot-config.h"

extern pros::Controller controller;
extern pros::Motor intakeF;
// motor groups
extern pros::Motor intakeM;
extern pros::Motor intakeB;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Imu imu;
extern pros::Rotation verticalEnc;
extern pros::Rotation horizontalEnc;
extern pros::ADIDigitalOut loader;
extern pros::ADIDigitalOut eliminate;
extern pros::ADIDigitalOut aligner;
extern pros::ADIDigitalOut wheelup;
extern pros::Optical optical;

//lemlib defs

extern lemlib::TrackingWheel horizontal;
extern lemlib::TrackingWheel vertical;
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern lemlib::OdomSensors sensors;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;
