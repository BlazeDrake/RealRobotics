#include "main.h"

#include "okapi/api.hpp"

#include "lib7842/api.hpp"



using namespace okapi;

using namespace lib7842;



//motor constants(if odom is used)

const int FrontLeft{20};
const int FrontRight{-19};


//controller stuff

Controller masterController;
ControllerDigital rollTop{ControllerDigital::R1};
ControllerDigital rollBot{ControllerDigital::R2};
ControllerDigital reverseTop{ControllerDigital::B};
ControllerDigital reverseBot{ControllerDigital::down};


ControllerDigital botIn{ControllerDigital::L1};
ControllerDigital botOut{ControllerDigital::L2};



//motor stuff
Motor LeftFront{20};
MotorGroup LeftDrive{FrontLeft,13};

MotorGroup RightDrive{FrontRight,-11};

Motor topRoller{2};
Motor botRoller{8};
Motor botIntakeLeft{6};
Motor botIntakeRight{-7};

//if pid is needed for 1 motor std::shared_ptr<Motor> ramp=std::make_shared<Motor>(rampPort);




//pid & odom stuff for when it's time to test PID auton

//odom(Change the values when bot is built)







std::shared_ptr<OdomChassisController> drive= ChassisControllerBuilder()
  .withMotors(LeftDrive,RightDrive)
  .withGains({.7,0.0,.00000}, {.003,0.0,0.0003})
  //.withSensors(leftEncoder,rightEncoder) //<-encoders
  .withDimensions(AbstractMotor::gearset::green,ChassisScales{{3.25_in,10.25_in},imev5GreenTPR})
  .withOdometry()
  .buildOdometry();
  //.withClosedLoopControllerTimeUtil(25,5,250_ms)










//auton select

std::shared_ptr<GUI::Screen> screen;
GUI::Selector* selector;

//Tray Pid









//other variables


int rollMultTop=1;
int rollMultBot=1;
bool pressingTop=false;
bool pressingBot=false;

const double driveSpeed(0.75);//<-percentage, 1=100%





//functions for my sanity

bool Dinput(ControllerDigital ibutton){

 return masterController.getDigital(ibutton);



}





void auton(int mult=1){

  //Move foward & grab cubes
/*
    drive->moveDistanceAsync(45_in);//<-move 2 sqaures

    take.moveVelocity(takeSpeed);

    drive->waitUntilSettled();

    take.moveVelocity(0);

    //return to start

    drive->moveDistance(-45_in);//<-move back 2 squares

    drive->waitUntilSettled();

    //turn

    drive->turnAngle(mult*90_deg);

    drive->waitUntilSettled();

    //move to scoring & stack

    drive->moveDistance(12_in);//<-move half a square

    drive->waitUntilSettled();

    tray->setTarget(rampTop);

    tray->waitUntilSettled();

    tray->setTarget(rampBottom);

    tray->waitUntilSettled();


*/
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
  //leftEncoder=LeftDrive.getEncoder();


	RightDrive.tarePosition();
	RightDrive.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
  //rightEncoder=RightDrive.getEncoder();










  //auton stuff

  screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(153, 157, 161) );

	screen->startTask("screenTask");



	selector = dynamic_cast<GUI::Selector*>(

    	&screen->makePage<GUI::Selector>("Selector")

			.button("Default", [&]() {

         //drive->moveDistance(-12_in);/*<-move half a square(push into small zone)*/

         //drive->moveDistance(12_in);/*<-move back*/
       })

      /*.button("Red", [&]() { auton(-1); })

      .button("Blue", [&]() { auton(); })*/

      .build()

    );

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

    //selector->run();

    drive->driveToPoint(Point{0_in,1_in});



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
	pros::lcd::set_text(1, "right!");


		//driving

    double left, right,

    forward(-masterController.getAnalog(ControllerAnalog::leftY)),

    turn(masterController.getAnalog(ControllerAnalog::rightX));



    if(std::abs(forward)<=0.1){

        left=-turn;

        right=turn;

    }

    else{

      left=forward-(0.75*turn);

      right=forward+(0.75*turn);

    }

    drive->getModel()->tank(left*driveSpeed,right*driveSpeed,.1);

    if(Dinput(rollTop)){
      topRoller.moveVelocity(600*rollMultTop);
    }
    else{
      topRoller.moveVelocity(0);
    }

    if(Dinput(rollBot)){
      botRoller.moveVelocity(600*rollMultBot);
    }
    else{
      botRoller.moveVelocity(0);
    }

    if(Dinput(botIn)){
      botIntakeLeft.moveVelocity(600);
      botIntakeRight.moveVelocity(600);
    }
    else if(Dinput(botOut)){
      botIntakeLeft.moveVelocity(-600);
      botIntakeRight.moveVelocity(-600);
    }
    else{
      botIntakeLeft.moveVelocity(0);
      botIntakeRight.moveVelocity(0);
    }

    if(Dinput(reverseTop)){
        if(!pressingTop){
          rollMultTop=-rollMultTop;
          pressingTop=true;
        }
    }
    else{
        pressingTop=false;
    }

    if(Dinput(reverseBot)){
        if(!pressingBot){
          rollMultBot=-rollMultBot;
          pressingBot=true;
        }
    }
    else{
        pressingBot=false;
    }
//auton button(COMMENT OUT FOR COMPS)

/*if(Dinput(ControllerDigital::A)){

  autonomous();

}

		pros::delay(20);

	}*/



}
}
