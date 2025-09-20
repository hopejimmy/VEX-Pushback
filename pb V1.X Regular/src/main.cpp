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
const char* titles[] = {"No Auto","SoloAwp","Left","Right"};
const char* teamcolor[]={"RED","Blue"};

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
    chassis.calibrate(); 
	chassis.setPose(0,0,0);
	optical.set_led_pwm(0);	
	controller.print(0, 0,"X: %.3f", chassis.getPose().x); // x
	pros::delay(50);
    controller.print(1,0, "Y: %.3f", chassis.getPose().y); // y
	pros::delay(50);
    controller.print(2,0, "Theta: %.3f", chassis.getPose().theta); // heading
	pros::delay(50);
	pros::Task screenTask1{[&]() {
		while(true){
			controller.clear();
			pros::lcd::print(1,"Battery: %.0f%%", pros::battery::get_capacity());
			pros::delay(50);
			pros::lcd::print(2, titles[autoSelect]);
			pros::lcd::print(3, teamcolor[team]);	
			pros::delay(50);
			controller.print(0, 0,"X: %.3f", chassis.getPose().x); // x
			pros::delay(50);
            controller.print(1,0, "Y: %.3f", chassis.getPose().y); // y
			pros::delay(50);
            controller.print(2,0, "Theta: %.3f", chassis.getPose().theta); // heading
			pros::delay(50);
	if ((pros::lcd::read_buttons()) & LCD_BTN_LEFT) {

			autoSelect++;			
            while ((pros::lcd::read_buttons()) & LCD_BTN_LEFT) {pros::delay(10);}    			
        }else  if ((pros::lcd::read_buttons()) & LCD_BTN_RIGHT) {

			autoSelect--;
            while ((pros::lcd::read_buttons()) & LCD_BTN_RIGHT) {pros::delay(10);}			
        }else  if ((pros::lcd::read_buttons()) & LCD_BTN_CENTER) {

			team++;
            while ((pros::lcd::read_buttons()) & LCD_BTN_CENTER) {pros::delay(10);}			
		}
		if (autoSelect ==4) {
		autoSelect = 0;
        }else if (autoSelect ==-1) {
		autoSelect = 3;
		}else if(team==2){
		team=0;
		}
            pros::delay(200);
	
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







void colorfind(){
	while(1){
	if(team==0){            //RED   
		color=red;
	}else if (team==1) {    //BLUE
		color=blue;
	}

	if((((optical.get_hue() > BLUE_LOWER_BOUND )&& (optical.get_hue() < BLUE_UPPER_BOUND))&&(color==red))||
	(((optical.get_hue() > RED_LOWER_BOUND )&&(optical.get_hue() < RED_UPPER_BOUND))&&(color==blue))){
		pros::delay(200);
		intakeB.move_velocity(-200);
		intakeF.move_velocity(600);
		pros::delay(1500);
	}else{
			intakeB.move_velocity(200);
			intakeF.move_velocity(600);
	}
	
}
}







void soloawp(){

// left start
	chassis.setPose(-13.476,1.68,0);
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
	//step 1
	chassis.moveToPose(-13.48,-28,0,900,{.forwards = false,.lead=0,.maxSpeed = 100,.minSpeed=60,.earlyExitRange = 0.3});
	chassis.turnToHeading(90,700,{.minSpeed = 20,.earlyExitRange = 0.1});
	//loading
	intaking(600,200,intake);// pros::Task colorf(colorfind);	
	loader.set_value(true);
	chassis.moveToPose(1,-28,90,900,{.lead = 0,.maxSpeed = 60,.minSpeed=50});
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.moveToPose(-33,-30,89 ,1300,{.forwards = false,.lead=0.1,.maxSpeed = 80,.minSpeed=70});
	pros::delay(200);
	loader.set_value(false);
	//aligner
	pros::delay(200);
	aligner.set_value(true);
	alignerActivate=true;
	//score long goal
	pros::delay(250);
	
	intaking(600,200,high);
	chassis.waitUntil(1);
	pros::delay(1700);


	intaking(600,200,intake);
	chassis.swingToHeading(-47,lemlib::DriveSide::LEFT,900,{.minSpeed = 70,.earlyExitRange = 0.2});
	chassis.moveToPose(-36,-6,-47,800,{.lead = 0,.maxSpeed = 90,.minSpeed = 50});
	chassis.turnToHeading(134, 900,{.maxSpeed=60,.minSpeed=40});
	chassis.waitUntilDone();
	chassis.moveToPose(-51.7,10.6,134,900,{.forwards = false,.lead=0,.maxSpeed = 80,.minSpeed = 60});
    pros::delay(800);
	intaking(600,80,mid);
	pros::delay(1500);




	intaking(600,200,intake);
	// score other long dih goal
	chassis.moveToPose(-29,7,135,500,{.lead = 0,.minSpeed = 80});
	chassis.swingToHeading(0,lemlib::DriveSide::LEFT,800,{.minSpeed = 90,.earlyExitRange = 0.5});
	chassis.moveToPose(-40,40,0,900,{.lead= 0,.maxSpeed=120,.minSpeed = 100,.earlyExitRange = 7});
	chassis.swingToHeading(50,lemlib::DriveSide::RIGHT,500,{.minSpeed = 70,.earlyExitRange = 0.9});
	chassis.moveToPoint(-12,66,700, {.maxSpeed = 110,.earlyExitRange = 2});
	chassis.waitUntilDone();
	chassis.turnToHeading(90, 500,{.maxSpeed=60,.minSpeed=40,.earlyExitRange=5});
	chassis.moveToPose(-35,66,90,800,{.forwards = false,.lead=0,.maxSpeed=90,.minSpeed = 60,.earlyExitRange = 0});
	pros::delay(500);
	intaking(600,200,high);
	pros::delay(400);
	wheelup.set_value(true);



}




void Right(){

	chassis.setPose(0,0,0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	intaking(600,200,intake);
	chassis.moveToPose(10.7, 24.2, 55, 1000,{.lead=-0.2,.minSpeed=50});
	chassis.turnToHeading(145,600);
	chassis.moveToPose(33, 5, 151,1200,{.lead = -0.2,.minSpeed = 50,.earlyExitRange=0.5});
	chassis.moveToPose(37.5, -18.3, 180, 1200,{.lead = 0,.maxSpeed=70,.minSpeed = 50});
	loader.set_value(true);
	chassis.waitUntilDone();
	chassis.moveToPose(37.5,17, 180,2000,{.forwards = false,.lead=0,.minSpeed=50,});
	pros::delay(100);
	loader.set_value(false);
	pros::delay(200);
	aligner.set_value(true);
	pros::delay(400);

	intaking(600,200,high);
	pros::delay(3000);
	chassis.moveToPose(37.5, -18.3, 180, 1200,{.lead = 0,.maxSpeed=70,.minSpeed = 50});
	intaking(600,200,removeloader);
	aligner.set_value(false);
	pros::delay(200);
	loader.set_value(true);
	loaderActivate=true;
	chassis.waitUntilDone();	
	wheelup.set_value(true);

}




void Left(){
	chassis.setPose(-13.476,1.68,0);
    chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
	//step 1
	chassis.moveToPose(-13.48,-28,0,1000,{.forwards = false,.lead=0,.maxSpeed = 100,.minSpeed=60,.earlyExitRange = 0.3});
	chassis.turnToHeading(90,700,{.minSpeed = 20,.earlyExitRange = 0.1});
	//loading
	intaking(600,200,intake);// pros::Task colorf(colorfind);	
	loader.set_value(true);
	chassis.moveToPose(0,-28,90,900,{.lead = 0,.maxSpeed = 60,.minSpeed=50});
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.moveToPose(-33,-30,89 ,1500,{.forwards = false,.lead=0.1,.maxSpeed = 80,.minSpeed=70});
	pros::delay(300);
	loader.set_value(false);
	//aligner
	pros::delay(200);
	aligner.set_value(true);
	//score long goal
	pros::delay(200);
	intaking(600,200,high);
	chassis.waitUntil(1);
	pros::delay(1700);
	chassis.swingToHeading(-41,lemlib::DriveSide::LEFT,1000,{.minSpeed = 50,.earlyExitRange = 0.2});
	aligner.set_value(false);
	intaking(600,200,intake);
	chassis.moveToPose(-34,-6,-43,1000,{.lead = 0,.maxSpeed = 90,.minSpeed = 40});
	chassis.turnToHeading(134, 800,{.maxSpeed=80,.minSpeed=40});
	chassis.moveToPose(-51.7,10.6,137,2500,{.forwards = false,.maxSpeed = 90,.minSpeed = 60});
	pros::delay(600);
	intaking(600,80,mid);
	pros::delay(1500);
	intaking(600,200,removeloader);
	chassis.moveToPose(-20, -24, 133, 1000,{.lead=0,.maxSpeed=100,.minSpeed = 40,.earlyExitRange =5});
	chassis.swingToHeading(91,lemlib::DriveSide::LEFT,500,{.minSpeed = 50,.earlyExitRange = 2});
	chassis.moveToPose(0,-28,90,2500,{.lead = 0,.maxSpeed = 70,.minSpeed=50});	
	loader.set_value(true);
    loaderActivate=true;
    pros::delay(1000);
	wheelup.set_value(true);	
	pros::delay(6000);


}



void Null(){
	chassis.setPose(0,0,0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST );
	chassis.moveToPose(0,-5,0,500,{.lead=0});
	chassis.waitUntilDone();
	wheelup.set_value(true);
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
optical.set_led_pwm(100);	
switch (autoSelect) {
        case 0:
            Null();
            break;
        case 1:
   	        soloawp();        
            break;

        case 2:
			Left();
            break;

		   case 3: 
			Right();
			break;

		
    }



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

int elimcon=0;
 

void opcontrol() {
	optical.set_led_pwm(0);	
	chassis.setBrakeMode(MOTOR_BRAKE_COAST);
    intakecoast();
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




if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
if(elimcon==0){
	elimcon=1;
}else{
	elimcon=0;
}
while(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))	{}
}

if(elimcon==0){
	eliminate.set_value(false);
}
if(elimcon==1){
	eliminate.set_value(true);
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