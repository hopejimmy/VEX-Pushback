#include "robot-config.h"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "main.h"
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//front roller
pros::Motor intakeF(6,pros::MotorGearset::blue);
//indexer
pros::Motor intakeM(-2,pros::MotorGearset::green);
//back roller
pros::Motor intakeB(12,pros::MotorGearset::green);
//drivetrain left
pros::MotorGroup leftMotors({-20, 18, -19},pros::MotorGearset::blue);
pros::MotorGroup rightMotors({10, -8, 9}, pros::MotorGearset::blue);
//odometry wheels
pros::Rotation verticalEnc(-5);
pros::Rotation horizontalEnc(15);
//pneumatics
pros::ADIDigitalOut loader(1);
pros::ADIDigitalOut eliminate(7);
pros::ADIDigitalOut wheelup(2);
pros::ADIDigitalOut aligner(8);

pros::Optical optical(13);

//imu
pros::Imu imu(7);
//lemlib defs
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -0.059);
lemlib::TrackingWheel horizontal(&horizontalEnc, 1.98, -1.93);


lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
    &rightMotors, // right motor group
    11.8, // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    400, // drivetrain rpm is 400
    4 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(
    13, // proportional gain (kP)
    0.12, // integral gain (kI)
    90, // derivative gain (kD)
    3, // anti wi ndup
    0, // small error range, in inches
    100, // small error range timeout, in milliseconds
    0, // large error range, in inches
    500, // large error range timeout, in milliseconds
    14// maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(3.3, // proportional gain (kP)
                                            0.001, // integral gain (kI)
                                            27, // derivative gain (kD)
                                            5, // anti windup
                                            0, // small error range, in degrees
                                            100, // small error range timeout, in milliseconds
                                            0, // large error range, in degrees
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);
// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     7, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
                                     //1.021
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                 5, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
                                  //1.021
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve

);