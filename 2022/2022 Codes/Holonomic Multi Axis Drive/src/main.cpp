/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Eli                                              */
/*    Created:      Fri Jan 28 2022                                           */
/*    Description:  V5 project                                                */
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
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "X-DriveFunctions/MultiAxisNavigation.cpp"
#include "AuxiliaryFunctions/CascadeLift.cpp"
#include "AuxiliaryFunctions/BaseLock.cpp"

using namespace vex;

competition Competition;

int Brain_precision = 0, Console_precision = 0, Controller1_precision = 0;

double CurrentXAxis, CurrentYAxis;
int LockDesiredState;

//Autonomous
int onauton_autonomous_0() {
  Gyroscope.startCalibration();
  while (Gyroscope.isCalibrating()) { task::sleep(50); }
  Gyroscope.setHeading(180.0, degrees);
  // pickup red
  SendLiftto(-600.0);
  Destination(3260.0, 550.0, 180.0, 0.6);
  LiftLeft.stop();
  LiftRight.stop();
  Lock_Base();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  SendLiftto(0.0);
  // drop red
  Waypoint(3260.0, 2000.0, 180.0, 0.7);
  Destination(2700.0, 2400.0, 90.0, 0.6);
  SendLiftto(-600.0);
  Unlock_Base();
  Destination(3220.0, 2400.0, 90.0, 0.7);
  // pickup blue
  Waypoint(3250.0, 3300.0, 90.0, 0.5);
  Destination(2935.0, 3300.0, 90.0, 0.4);
  Lock_Base();
  SendLiftto(600.0);
  Waypoint(3250.0, 3220.0, 90.0, 0.5);
  Waypoint(2820.0, 2400.0, 90.0, 0.7);
  // bulldoze yellow 1
  Waypoint(2820.0, 900.0, 90.0, 0.7);
  Destination(2820.0, 1100.0, 90.0, 0.7);
  // spin move blue
  Unlock_Base();
  SendLiftto(-600.0);
  Waypoint(2500.0, 1100.0, 300.0, 0.7);
  Waypoint(1800.0, 1100.0, 270.0, 0.7);
  // bulldoze yellow 2
  Waypoint(1780.0, 1300.0, 270.0, 0.7);
  Waypoint(1780.0, 2300.0, 270.0, 0.7);
  Waypoint(1300.0, 2300.0, 270.0, 0.7);
  // pickup blue
  Waypoint(1200.0, 2775.0, 90.0, 0.7);
  Destination(790.0, 2775.0, 90.0, 0.4);
  Lock_Base();
  SendLiftto(1.0);
  // yellow number 3
  Waypoint(1000.0, 2300.0, 270.0, 0.7);
  Waypoint(1000.0, 1300.0, 270.0, 0.7);
  // drop blue 2
  Destination(1400.0, 1350.0, 300.0, 0.4);
  Unlock_Base();
  SendLiftto(-600.0);
  Waypoint(1130.0, 1350.0, 270.0, 0.6);
  Waypoint(350.0, 1350.0, 270.0, 0.5);
  // pick red
  Waypoint(350.0, 450.0, 270.0, 0.5);
  Destination(650.0, 450.0, 270.0, 0.4);
  Lock_Base();
  SendLiftto(600.0);
  Waypoint(350.0, 450.0, 270.0, 0.7);
  // yeet
  Destination(430.0, 2600.0, 270.0, 0.7);
  Unlock_Base();
  SendLiftto(-600.0);
  return 0;
}

//Driver Control
int ondriver_drivercontrol_0() {
  while (true) {
    ManualMotors();
    ManualLift();
    ManualLockToggle();
  wait(5, msec);
  }
  return 0;
}

//Initalization
int whenStarted1() {
  Set_Offset(3355.0, 230.0);
  Console_precision = -1;
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
  BaseLock.setStopping(brake);
  return 0;
}




void VEXcode_driver_task() {
  // Start the driver control tasks....
  vex::task drive0(ondriver_drivercontrol_0);
  while(Competition.isDriverControl() && Competition.isEnabled()) {this_thread::sleep_for(10);}
  drive0.stop();
  return;
}

void VEXcode_auton_task() {
  // Start the auton control tasks....
  vex::task auto0(onauton_autonomous_0);
  while(Competition.isAutonomous() && Competition.isEnabled()) {this_thread::sleep_for(10);}
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
