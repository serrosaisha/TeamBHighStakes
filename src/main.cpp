/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*--------------
--------------------------------------------------------------*/


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                   
// Drivetrain           drivetrain    1, 10, D      ` 
// ClawMotor            motor         3              
// ArmMotor             motor         8              
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"


using namespace vex;


// A global instance of competition
competition Competition;


// define your global instances of motors and other devices here


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


void pre_auton(void) {
 // Initializing Robot Configuration. DO NOT REMOVE!
 vexcodeInit();
  // All activities that occur before the competition starts
 // Example: clearing encoders, setting servo positions, ...
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*----------------------------- ----------------------------------------------*/
double kp = 0.2;
double ki = 0;
double kd = 0;

void pid(double targetDistance) {
  double error = targetDistance;
  double integral = 0;
  double lastError =  targetDistance;
  double prevDistanceError = fl.position(degrees);
  fl.setPosition(0, degrees);
  ml.setPosition(0, degrees);
  bl.setPosition(0, degrees);
  fr.setPosition(0, degrees);
  mr.setPosition(0, degrees);
  br.setPosition(0, degrees);
  while (true) {
    double measureDistance = (fl.position(degrees) + fr.position(degrees))/2;
    error = targetDistance - measureDistance;
    prevDistanceError = measureDistance;
    if (fabs(error)<30) {
      fl.stop(brake);
      ml.stop(brake);
      bl.stop(brake);

      fr.stop(brake);
      mr.stop(brake);
      br.stop(brake);
      return;
    }
    
    double speed = error * kp + integral * ki + (error - lastError) * kd;
    fl.spin(fwd, speed, percent);
    ml.spin(fwd, speed, percent);
    bl.spin(fwd, speed, percent);

    fr.spin(fwd, speed, percent);
    mr.spin(fwd, speed, percent);
    br.spin(fwd, speed, percent);

    lastError = error;
    wait(15, msec);
  }
}
 
void clamp() {
 mogo.set(true);
 mogo2.set(true);
}


void unclamp() {
 mogo.set(false);
 mogo2.set(false);
}

void moveAllWheels(int SpeedLeft, int SpeedRight, int ) {
 fl.spin(forward, SpeedLeft + SpeedRight, percent);
 ml.spin(forward, SpeedLeft + SpeedRight, percent);
 bl.spin(forward, SpeedLeft + SpeedRight, percent);


 fr.spin(reverse, SpeedLeft - SpeedRight, percent);
 mr.spin(reverse, SpeedLeft - SpeedRight, percent);
 br.spin(reverse, SpeedLeft - SpeedRight, percent);
}


void turnLeft(double angle) {
 // basically the same as right except left motor spins reverse and right is forward
 inertialSensor.setRotation(0, degrees);

 //turning left using inertial sensor
 while (fabs(inertialSensor.rotation(deg)) < angle) {
   double diff =  angle - fabs(inertialSensor.rotation(deg));
   // 5 + diff * 0.3 ,pct means to slow down when reaching the precent target.
   //You have to remember to set the minimum speed to 5 so it does not slowly move
   fr.spin(forward, 5 + diff * 0.3, pct);
   mr.spin(forward, 5 + diff * 0.3, pct);
   br.spin(forward, 5 + diff * 0.3, pct);
   
   fl.spin(reverse, 5 + diff * 0.3, pct);
   ml.spin(reverse, 5 + diff * 0.3, pct);
   bl.spin(reverse, 5 + diff * 0.3, pct);
   wait(1, msec);
 }
}

void intaking() {
 if (controller1.ButtonR2.pressing()) {
   intake.spin(forward, 350, rpm);
   intake2.spin(reverse, 400, rpm);
 } else if (controller1.ButtonR1.pressing()) {
   intake.spin(reverse, 80, pct);
   intake2.spin(fwd, 90, percent);
 } else {
   intake.stop(coast);
   intake2.stop(coast);
 }
}

void turnRight(double angle) {
 // set inertial rotation to 0 degrees
 inertialSensor.setRotation(0, degrees);


 //turn right using inertial sensors
 while (inertialSensor.rotation(deg) < angle) {
   double diff =  angle - fabs(inertialSensor.rotation(deg));
   fl.spin(forward, 5 + diff * 0.3, pct);
   ml.spin(forward, 5 + diff * 0.3, pct);
   bl.spin(forward, 5 + diff * 0.3, pct);
   fr.spin(reverse, 5 + diff * 0.3, pct);
   mr.spin(reverse, 5 + diff * 0.3, pct);
   br.spin(reverse, 5 + diff * 0.3, pct);
 }
}

void setVelocity(double vel) {
 // set all motors to velocity value of 'vel'
 fl.setVelocity(vel, percent);
 ml.setVelocity(vel, percent);
 bl.setVelocity(vel, percent);
 fr.setVelocity(vel, percent);
 mr.setVelocity(vel, percent);
 br.setVelocity(vel, percent);
}

void stopWheels() {
 // stop all motors in brake
 fl.stop(brake);
 ml.stop(brake);
 bl.stop(brake);


 fr.stop(brake);
 mr.stop(brake);
 br.stop(brake);
}

void auton1() {
  pid(-750);
  clamp();
  intake.spin(fwd, 90, pct);
  wait(6, sec);
  unclamp();
}

void auton2() {

  // pid(900);
  // turnRight(90);
  // pid(900);
  // turnRight(90);
  // pid(900);
  // turnRight(90);
  // pid(900);
  // turnRight(90);
  // pid(5);

  pid(900);
  turnLeft(90);
  pid(900);
  turnLeft(90);
  pid(900);
  turnLeft(90);
  pid(900);
  turnLeft(90);
  pid(5);


  // pid(900);
  // turnRight(40);
  // pid(1200);
  // turnLeft(45);
  // pid(900);
  // turnLeft(90);
  // pid(900);
  // turnLeft(90);
  // pid(900);
  // turnLeft(45);
  // pid(1200);
  // turnRight(45);
  // pid(900);
  // turnRight(90);
  // pid(900);
  // pid(500);
  // turnLeft(90);
  // // pid(400);
}

void autonomous(void) {
  auton2();
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void old_arcade() {
 //Slower
 // int speedleft = controller1.Axis1.value()/2;
 // int speedright = controller1.Axis3.value()/2;


 double speedleft = controller1.Axis1.value() + controller1.Axis3.value();
 double speedright = controller1.Axis1.value() - controller1.Axis3.value();


 fl.spin(forward, speedleft, percent);
 ml.spin(forward, speedleft, percent);
 bl.spin(forward, speedleft, percent);


 // RIGHT MOTORS ARE REVERSED SO FORWARD = REVERSE!!!!!!!!!
 fr.spin(reverse, speedright, percent);
 mr.spin(reverse, speedright, percent);
 br.spin(reverse, speedright, percent);
}



void stopAllWheels(){
 fl.stop();
 ml.stop();
 bl.stop();
 fr.stop();
 mr.stop();
 br.stop();
}

//logic statement
bool clamptrue = false;
bool prevclamp = false;
void usercontrol() {
  while (1) {
    intaking();
    old_arcade();
    if (controller1.ButtonL1.pressing()) {
      if (prevclamp == false) {
        prevclamp = true;
      }
    } else {
      if (prevclamp == true) {
        prevclamp = false;
      }
    }
    if(clamptrue){
      clamp();
    } else if(!clamptrue){
      unclamp();
    }
  wait(20,msec);
  }
}

// Main will set up the competition functions and callbacks.
//
int main() {
 // Set up callbacks for autonomous and driver control periods.
 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);


 // Run the pre-autonomous function.
 pre_auton();
 inertialSensor.calibrate();
 while(inertialSensor.isCalibrating()) {
  wait(20, msec);
 }
 // Prevent main from exiting with an infinite loop.
 while (true) {
   wait(100, msec);
 }
}





