#include "main.h"
#include "okapi/api.hpp"

using namespace okapi;

//motor constants(if odom is used)
//const int FrontLeft=6;
//const int FrontRight=9;
//controller stuff
Controller masterController;
ControllerDigital rampUp=ControllerDigital::L2;
ControllerDigital rampDown=ControllerDigital::L1;

ControllerDigital TakeIn=ControllerDigital::R1;
ControllerDigital TakeOut=ControllerDigital::R2;
//Scale for auton
ChassisScales Scales={{3.25_in,10.25_in},imev5GreenTPR};

//motor stuff
MotorGroup LeftDrive={-11,3};
MotorGroup RightDrive={-20,-5};

Motor ramp(7);

MotorGroup take({9,-10});


//pid & odom stuff for when it's time to test PID auton
//odom(Change the values when bot is built)
/*
IntegratedEncoder left(FrontLeft,false);
ADIEncoder mid(4,3);
IntegratedEncoder right(FrontRight,false);*/


//Pid(Only use a PD controller), will probably delete
//auto PID= IterativeControllerFactory::posPID(0.001, 0.0, 0.000);
//IterativePosPIDController::Gains pos{.002,.0000,.00003,.00};//<-position(KU:unknown,PU:unknown)
//IterativePosPIDController::Gains angle={.00/*30*/,.0000,.0000,.00};/*<-keeping it straight(don't use yet)*/
//IterativePosPIDController::Gains turn={.0035,.0000,.00015,.00};/*<-turning(don't use yet)*/

auto drive= ChassisControllerBuilder()
  .withMotors(LeftDrive,RightDrive)
  //.withSensors(left,right,mid) //<-encoders
  .withDimensions(AbstractMotor::gearset::green,Scales)
  .withClosedLoopControllerTimeUtil(25,5,250_ms)
  //.withGains(pos,turn,angle)
  .build();


//other variables
int rampSpeed=40;
int takeSpeed=200;
bool constantIntake=false;
bool checking=false;

//functions for my sanity
bool Dinput(ControllerDigital ibutton){
 return masterController.getDigital(ibutton);
}




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}

}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	//odometer initialization
	LeftDrive.tarePosition();
  LeftDrive.setEncoderUnits(AbstractMotor::encoderUnits::rotations);

	RightDrive.tarePosition();
	RightDrive.setEncoderUnits(AbstractMotor::encoderUnits::rotations);

  take.tarePosition();
  take.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
  take.setGearing(AbstractMotor::gearset::green);

  ramp.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
  ramp.setGearing(AbstractMotor::gearset::red);
  //auton stuff
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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

  /*normal
  auton.generatePath({Point{10_in,0_in,0_deg}}, "red_a_1");
  auton.generatePath({Point{0_in,0_in,0_deg}}, "red_a_2");
  auton.setTarget("red_a_1",true);
  auton.waitUntilSettled();
  auton.setTarget("red_a_2",false);
  auton.waitUntilSettled();
  */

  //PID auton(for when it's time to do it
  //drive->moveDistance(10_in);
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

//vars at the top
void opcontrol() {


	while (true) {

		//UPDATE VERSION EVERY TIME PROGRAM IS CHANGED SO UPLOAD ISSUES ARE KNOWN!!!
   	pros::lcd::print(0,"Drive 0.7.8 Dev");

		//driving
    /*old drive
    drive->getModel()->arcade(masterController.getAnalog(ControllerAnalog::leftY),
						 masterController.getAnalog(ControllerAnalog::rightX));*/

    //new drive(look at muphries one for a basis)
    double left, right,
    turn=masterController.getAnalog(ControllerAnalog::rightX),
    forward=masterController.getAnalog(ControllerAnalog::leftY);

    if(std::abs(forward)<=0.1){
        left=turn;
        right=-turn;
    }
    else{
      left=forward+(0.75*turn);
      right=forward-(0.75*turn);
    }

    drive->getModel()->tank(left,right,.1);
	  //moving the ramp
		if(Dinput(rampUp)){
			ramp.moveVelocity(rampSpeed);
		}
		else if(Dinput(rampDown)){
			ramp.moveVelocity(-rampSpeed);
		}
		else{
			ramp.moveVelocity(0);
		}
		pros::delay(20);
		//intake/outtake
		if(Dinput(TakeIn)||Dinput(TakeOut)){
			pros::delay(100);
			if(Dinput(TakeIn)&&Dinput(TakeOut)&&!checking){
					constantIntake=!constantIntake;
          checking=true;
			}
		else{
      checking=false;
    }
	}
  if((Dinput(TakeIn)||constantIntake)&&!Dinput(TakeOut)){
      take.moveVelocity(takeSpeed);
  }
  else if(Dinput(TakeOut)&&!constantIntake){
      take.moveVelocity(-takeSpeed);
  }
  else{
      take.moveVelocity(0);
  }

		pros::delay(20);
	}

}
