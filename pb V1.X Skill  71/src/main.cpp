#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "movepid.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "robot-config.h"
#include <string>
    
#define RED_UPPER_BOUND  30
#define RED_LOWER_BOUND  0

#define BLUE_UPPER_BOUND 230
#define BLUE_LOWER_BOUND 170


std::string red = "Red";
std::string blue = "Blue";

std::string high = "High";
std::string mid = "Mid";
std::string intake = "Intaking";
std::string removeloader = "Remove";
std::string color;


int team=0;

int autoSelect = 0;
bool auto_started= false;

void intakecoast(){
	intakeB.set_brake_mode(pros::MotorBrake::coast);
	intakeM.set_brake_mode(pros::MotorBrake::coast);
	intakeF.set_brake_mode(pros::MotorBrake::coast);
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


void initialize() {
	pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors
	chassis.setPose(0,0,0);//set [pose]
	controller.clear();
	pros::Task screenTask{[&]() {
        while (true) {
            // print robot location to the brain screen
			optical.get_hue();
			intakeM.get_current_draw();
			// autoSelector();

			controller.print(0,0,"X: %.3f", chassis.getPose().x);
			pros::delay(50);
			controller.print(1,0,"Y: %.3f", chassis.getPose().y);
			pros::delay(50);
			controller.print(2,0,"Theta: %.3f", chassis.getPose().theta);
			pros::delay(50);


			pros::delay(20);
			// red = 330- 350
			// blue = 210 - 230
			
        }
    }};


}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}
/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}



bool colorRed = false;
bool colorBlue = false;
static int64_t startTime = 0;
bool triggered = false;
// int proximity = optical.get_proximity();

bool alignerActivate = false;
bool loaderActivate = false;
bool loadLatch = false;
bool alignLatch = false;

bool detected = false;



bool is_red() {
    return ((optical.get_hue() > RED_LOWER_BOUND )&&(optical.get_hue() < RED_UPPER_BOUND)); 
}

bool is_blue() {
    return ((optical.get_hue() > BLUE_LOWER_BOUND )&& (optical.get_hue() < BLUE_UPPER_BOUND));
}

bool is_intake_jammed() {
    return (intakeM.get_current_draw() >= 1800);
}

bool is_intake_jammed_mid() {
    return (intakeB.get_current_draw() >= 1500);
}

bool both_jammed(){
	if(is_intake_jammed ()&& is_intake_jammed_mid())
	return true;

	else 
	return false;
}

void intaking(int velocity_11, int velocity5_5, std::string state){
	if (state == high){
		if(!is_intake_jammed()){
			intakeF.move_velocity(velocity_11);
			intakeM.move_velocity(velocity5_5);
			intakeB.move_velocity(velocity5_5);
		} else {
			intakeF.move_velocity(velocity_11);
			intakeM.move_velocity(-200);
			intakeB.move_velocity(velocity5_5);
		}
	}
	else if(state == mid){
			if(!is_intake_jammed()){
				intakeF.move_velocity(velocity_11);
				intakeM.move_velocity(150);
				intakeB.move_velocity(-velocity5_5);
			}
			else{
				intakeF.move_velocity(velocity_11);
				intakeM.move_velocity(-200);
				intakeB.move_velocity(-velocity5_5);
			}
			
		}
	else if(state == intake){
			intakeF.move_velocity(velocity_11);
			intakeM.move_velocity(0);
			intakeB.move_velocity(velocity5_5);
	}
	else if(state==removeloader){
			intakeF.move_velocity(velocity_11);
			intakeM.move_velocity(-200 );
			intakeB.move_velocity(-velocity5_5);
	}
}



void score(){
	pros::delay(1000);
	intaking(-600,200,high);
	pros::delay(300);
	intaking(600,200,high);	
	pros::delay(2500);
}











void skills(){
 chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
	chassis.setPose(-13.476,1.68,0);
 //reset

	chassis.moveToPose(-13.48,-28,0,900,{.forwards = false,.lead=0,.maxSpeed = 100,.minSpeed=60,.earlyExitRange = 0.3});
	chassis.turnToHeading(90,700,{.minSpeed = 20,.earlyExitRange = 0.1});
	//loading
	intaking(600,200,intake);// pros::Task colorf(colorfind);	
	loader.set_value(true);
	chassis.moveToPose(1,-29,90,900,{.lead = 0,.maxSpeed = 50,.minSpeed=50});
	chassis.waitUntilDone();
	pros::delay(1000);
	chassis.moveToPose(-33,-30,89 ,1300,{.forwards = false,.lead=0.1,.maxSpeed = 80,.minSpeed=70});
	pros::delay(200);
	loader.set_value(false); 
	//aligner
	pros::delay(200);
	aligner.set_value(true);
	//score long goal
	pros::delay(250);
	
	intaking(600,200,high);
	chassis.waitUntil(1);
score();


	intaking(600, 200, intake);
	aligner.set_value(false);


	chassis.moveToPose(-15, -29, 90, 1000,{.minSpeed = 60, .earlyExitRange = 0.3});
	chassis.turnToHeading(219, 800, {.minSpeed = 20, .earlyExitRange = 0.1});
	chassis.moveToPose(-28.5, -42, 220, 1000, {.minSpeed = 80, .earlyExitRange = 0.3});
	chassis.moveToPose(-105, -45, 268, 2000, {.minSpeed = 80, .earlyExitRange = 2});
	chassis.waitUntilDone();
	pros::delay(200);
	chassis.turnToHeading(360, 700, {.minSpeed = 20, .earlyExitRange = 0.1});
	chassis.waitUntilDone();
	pros::delay(200);
	loader.set_value(true);

	runtime(-50, 360, 500, 0.5);

	pros::delay(500);
	chassis.setPose(0, 0, 0);
	chassis.moveToPose(0, 16, 0, 800, {.lead = 0., .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.3});
	chassis.turnToHeading(-90,700,{.minSpeed = 20,.earlyExitRange = 0.1});
	// //loader 2
	chassis.moveToPose(-14, 17,  -90, 1500, {.lead = 0, .maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 0.7});
	pros::delay(2500);
	chassis.moveToPose(21, 16, -89, 1500, {.forwards = false,.lead=0.1,.maxSpeed = 70, .minSpeed = 60, .earlyExitRange = 0.6});

	pros::delay(300);
	loader.set_value(false);
	pros::delay(200);	
	aligner.set_value(true);
	pros::delay(500);
	intaking(600, 200, high);

score();
	// scoring 2


	

	intaking(600, 200, intake);
	chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
	chassis.moveToPose(0, 8, 0,500, {.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 0.7});
	chassis.turnToHeading(-90, 700, {.minSpeed = 20, .earlyExitRange = 0.3});
	aligner.set_value(false);
	chassis.moveToPose(100, 9, -90,  3300, {.forwards=false,.maxSpeed = 100, .minSpeed = 60, .earlyExitRange = 3});
	chassis.moveToPose(120, 9, -89,  800, {.forwards=false,.maxSpeed = 70, .minSpeed = 40, .earlyExitRange = 2});
	pros::delay(500);
	loader.set_value(true);
	chassis.waitUntilDone();
	// moving to 3rd quadrant

	chassis.setPose(0, 0, 0);
	pros::delay(500);
	chassis.moveToPose(0, 15, 0, 1000, {.lead = 0., .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 0.3});

	chassis.moveToPose(31, 16.5,  90, 2000, {.lead = 0, .maxSpeed = 60, .minSpeed = 55, .earlyExitRange = 0.7});
	chassis.waitUntilDone();
	pros::delay(1000);
	//loader 3
	chassis.moveToPose(-14.5, 16, 91, 1600, {.forwards = false,.lead=0.1,.maxSpeed = 70, .minSpeed = 60, .earlyExitRange = 0.6});
	pros::delay(200);
	loader.set_value(false);
	pros::delay(200);
	aligner.set_value(true);
	pros::delay(600);
	intaking(600, 200, high);
score();
	//scoring 3





	intaking(600, 200, intake);
	chassis.setPose(-33,-30,89);
	aligner.set_value(false);


	chassis.moveToPose(-25,-30,89,400,{.lead = 0,.maxSpeed = 90,.minSpeed = 60});
	chassis.swingToHeading(-55,lemlib::DriveSide::LEFT,800,{.minSpeed = 70,.earlyExitRange = 0.2});
	chassis.moveToPose(-44,-4,-60,1100,{.lead = 0,.maxSpeed = 60,.minSpeed = 60});
	chassis.turnToHeading(-224, 900,{.maxSpeed=60,.minSpeed=40});
	chassis.waitUntilDone();
	chassis.moveToPose(-54,7.6,-225,900,{.forwards = false,.lead=0,.maxSpeed = 80,.minSpeed = 60});
    pros::delay(800);
	intaking(600,80,mid);
	pros::delay(2000);
	intaking(600,200,intake);
	chassis.moveToPose(-40,-0.78,-223,500,{.lead = 0,.minSpeed = 80});
	chassis.swingToHeading(-90,lemlib::DriveSide::RIGHT,800,{.minSpeed = 80,.earlyExitRange = 3});
	//center goal




	chassis.moveToPose(-87, -3, -90, 1500, {.maxSpeed=90,.minSpeed = 80, .earlyExitRange = 2});
	aligner.set_value(true);

	chassis.turnToHeading(-130, 500, {.maxSpeed=60,.minSpeed = 30, .earlyExitRange = 1});
	chassis.moveToPoint(-110,-26,900, {.maxSpeed = 100,.earlyExitRange = 2});
	chassis.waitUntilDone();
	chassis.turnToHeading(-89, 500,{.maxSpeed=60,.minSpeed=40,.earlyExitRange=5});
	chassis.moveToPose(-98.5,-29.5,-89,800,{.forwards = false,.lead=0,.maxSpeed=90,.minSpeed = 60,.earlyExitRange = 0});
	pros::delay(300);
	intaking(600,200,high);
	pros::delay(1500);

	//move to loader 4











intaking(600,200,intake);
	chassis.moveToPose(-132, -29,  -90, 2000, {.lead=0.1, .maxSpeed = 60, .minSpeed = 55, .earlyExitRange = 0.7});
	aligner.set_value(false);
	pros::delay(200);
	loader.set_value(true);
	pros::delay(2700);
	//loader 4	

	chassis.moveToPose(-95,-29.5, -89, 1500, {.forwards = false,.lead=0.1,.maxSpeed = 80, .minSpeed = 60, .earlyExitRange = 0.6});

	pros::delay(300);
	loader.set_value(false);
	pros::delay(300);	
	aligner.set_value(true);
	pros::delay(500);
	intaking(600, 200, high);

score();

	// scoring 4
	intaking(-600, 200, intake);
	chassis.setPose(0, 0,0);
	chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
	aligner.set_value(false);
	chassis.moveToPose(5, 25, 20,500, {.lead = 0.2, .minSpeed = 50, .earlyExitRange = 5});
	chassis.moveToPose(36, 31, 88,700, {.lead = 0.5, .maxSpeed=120,.minSpeed = 80, .earlyExitRange = 3});
	chassis.moveToPose(50, 31, 89,1500, {.lead = 0.1, .maxSpeed=110,.minSpeed = 90, .earlyExitRange = 3});
	wheelup.set_value(true);
	pros::delay(200);
	loader.set_value(true);
	pros::delay(800);
	loader.set_value(false);
	//parking


}





 
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */



void autonomous() {
auto_started=true;

skills();



}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void alignerChange(bool state) {
	alignerActivate = state;
	aligner.set_value(state);
}

void loaderChange(bool state) {
	loaderActivate = state;
	loader.set_value(state);
}

void setMechanismStates(bool alignerState, bool loaderState) {
	if (!loaderState && loaderActivate) {
		loaderChange(false);
		pros::delay(200);
	}
	if (!alignerState && alignerActivate) {
		alignerChange(false);
		pros::delay(200);
	}
	if (alignerState && !alignerActivate) {
		alignerChange(true);
	}
	if (loaderState && !loaderActivate) {
		loaderChange(true);
	}
}

 



void opcontrol() {
 	optical.set_led_pwm(0);	
	wheelup.set_value(true);
    intakecoast();	
	chassis.setBrakeMode(MOTOR_BRAKE_COAST);
	while (true) {
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.arcade(leftY, rightX * 0.85);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intaking(600, 200, intake);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intaking(-600, -200, intake);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intaking(600, 200, high);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intaking(600, 120, mid);
		}else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			intaking(600,200,removeloader);
		  }
		else {
			intakeF.move_velocity(0);
			intakeB.move_velocity(0);
			intakeM.move_velocity(0);
		}


if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
	wheelup.set_value(true);
}
 


		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			if (!alignLatch) {
				alignLatch = true;
				setMechanismStates(!alignerActivate, false);
			}
		} else {
			alignLatch = false;
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			if (!loadLatch) {
				loadLatch = true;
				setMechanismStates(false, !loaderActivate);
			}
		} else {
			loadLatch = false;
		}

		pros::delay(10);
	}
}