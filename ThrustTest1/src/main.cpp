/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Eli                                              */
/*    Created:      Feb 1 2022                                                */
/*    Description:  multi autonomous codes                                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         19              
// RL                   motor         20              
// FR                   motor         9               
// RR                   motor         10              
// Gyroscope            inertial      18              
// y                    encoder       A, B            
// x                    encoder       C, D            
// LiftLeft             motor         7               
// LiftRight            motor         6               
// BaseLock             motor         17              
// DigitalOutH          digital_out   H               
// BackMotor            motor         16              
// DigitalOutG          digital_out   G               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "\..\BaseCode\AutonomousCodes.cpp"

using namespace vex;

competition Competition;

double lbs_mass = 5; //robot in lbs
double mass = lbs_mass * 0.453592;

//Odometry
double CurrentXAxis, CurrentYAxis, PreviousTheta, PreviousYValue, PreviousXValue, globaldelta;
//Vector Engine
double CurrentXVelocity, CurrentYVelocity, CurrentRVelocity;

//old
int LockDesiredState, BackDesiredState, BackGripperDesiredState, LeftInital, RightInital, BaseLockOffset;

// Autonomousv2
int onauton_autonomous_0() {
    ThrustTest();
return 0;
}

// Driver Control
int ondriver_drivercontrol_0() {
  while (true) {
    ThrustTest();
    wait(5, msec);
  }
}

// Initalization
int whenStarted1() {
  Gyroscope.startCalibration();
  Set_Offset(3355.0, 230.0);
  PreviousTheta = 180;
  PreviousYValue = 0;
  PreviousXValue = 0;
  LockDesiredState = -99;
  FL.setStopping(brake);
  RL.setStopping(brake);
  FR.setStopping(brake);
  RR.setStopping(brake);
  RL.setMaxTorque(100.0, percent);
  FR.setMaxTorque(100.0, percent);
  FL.setMaxTorque(100.0, percent);
  RR.setMaxTorque(100.0, percent);
  LiftLeft.setMaxTorque(100.0, percent);
  LiftRight.setMaxTorque(100.0, percent);
  LiftLeft.setVelocity(100.0, percent);
  LiftRight.setVelocity(100.0, percent);
  LiftLeft.setStopping(brake);
  LiftRight.setStopping(brake);
  BaseLock.setVelocity(100.0, percent);
  BaseLock.setStopping(hold);
  BackMotor.setVelocity(100, percent);
  BackMotor.setStopping(hold);
  LeftInital = LiftLeft.position(degrees);
  RightInital = LiftRight.position(degrees);
  return 0;
}

void VEXcode_driver_task() {
  // Start the driver control tasks....
  vex::task drive0(ondriver_drivercontrol_0);
  while (Competition.isDriverControl() && Competition.isEnabled()) {
    this_thread::sleep_for(10);
  }
  drive0.stop();
  return;
}

void VEXcode_auton_task() {
  // Start the auton control tasks....
  vex::task auto0(onauton_autonomous_0);
  while (Competition.isAutonomous() && Competition.isEnabled()) {
    this_thread::sleep_for(10);
  }
  auto0.stop();
  return;
}

int main() {
  vexcodeInit();
  vex::competition::bStopTasksBetweenModes = false;
  Competition.autonomous(VEXcode_auton_task);
  Competition.drivercontrol(VEXcode_driver_task);

  whenStarted1();
}


typedef struct{
int mass = 5; //kg
double CurrentXAxis = 0;
double CurrentYAxis = 0;
double CurrentTheta = 0;

double TargetXAxis = 0;
double TargetYAxis = 0;
double TargetTheta = 0;

double CurrentXEncoderValue = 0;
double CurrentYEncoderValue = 0;

double CurrentXVelocity = 0;
double CurrentYVelocity = 0;
double CurrentRVelocit = 0;

double MaxXVelocity = 0;
double MaxYVelocity = 0;
double MaxRVelocit = 0;
}robot;