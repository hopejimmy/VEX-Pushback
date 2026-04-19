#include "robot-config.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/distance.hpp"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//front roller
pros::Motor intakeF(14,pros::MotorGearset::blue);

//middle roller
pros::Motor intakeM(-3,pros::MotorGearset::green);

//back roller
pros::Motor intakeB(4,pros::MotorGearset::green);

//drivetrain
pros::MotorGroup leftMotors({8, -9, -10},pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-18, 19, 20}, pros::MotorGearset::blue);


//pneumatics
pros::ADIDigitalOut Odom(2);
pros::ADIDigitalOut loader(4);
pros::ADIDigitalOut hook(6);
pros::ADIDigitalOut highscore(3);
pros::ADIDigitalOut intakelift(5);

//intake sensors
pros::Optical optical_top(5);
pros::Optical optical_preload(2);
pros::Distance dis_preload(1 );
pros::Distance dis_roller(6);

pros::Distance distanceFront(11);
pros::Distance distanceLeft(7);
pros::Distance distanceRight(12);






//odometry wheel sensors
pros::Rotation verticalEnc(17);
pros::Rotation horizontalEnc(15);

//imu
pros::Imu imu(16);

//lemlib defs
lemlib::TrackingWheel vertical(&verticalEnc, 1.98, 0);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.4);


lemlib::Drivetrain drivetrain(
    &leftMotors, // left motor group
    &rightMotors, // right motor group
    11.34, // 11.34 inch track width
    lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(
    18, // proportional gain (kP)
    0.001, // integral gain (kI)
    55, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    0, // small error range timeout, in milliseconds
    0, // large error range, in inches
    0, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(
    2.5, // proportional gain (kP)
    0.001, // integral gain (kI)
    15, // derivative gain (kD)
    8, // intergral anti windup
    0, // small error range, in degrees
    0, // small error range timeout, in milliseconds
    0, // large error range, in degrees
    0, // large error range timeout, in milliseconds
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
lemlib::ExpoDriveCurve throttleCurve(5, // joystick deadband out of 127
                                     3, // minimum output where drivetrain will move out of 127
                                     0.999 // expo curve gain
                                     //1.021
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                 3, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
                                  //1.021
); 

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve

);


// //MCL sensors
// pros::Distance dNorth(11);//3.42, 5.12
// pros::Distance dNorthW(13);
// pros::Distance dWest(7);//-5.67, -3.86
// pros::Distance dEast(12);//5.67, -3.86
