// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         1               
// RL                   motor         6               
// FR                   motor         9               
// RR                   motor         10              
// Gyroscope            inertial      4               
// y                    encoder       C, D            
// x                    encoder       A, B            
// Flywheel1            motor         2               
// Flywheel2            motor         5               
// Intake               motor         14              
// Trigger              digital_out   H               
// EndgameLaunch        digital_out   G               
// Roller               motor         19              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         1               
// RL                   motor         6               
// FR                   motor         9               
// RR                   motor         10              
// Gyroscope            inertial      4               
// y                    encoder       C, D            
// x                    encoder       A, B            
// Flywheel1            motor         2               
// Flywheel2            motor         5               
// Intake               motor         14              
// Trigger              digital_out   H               
// EndgameLaunch        digital_out   G               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include "C:\Users\elimb\Documents\GitHub\BisonTech\BaseCode\AutonomousCodes.cpp"

using namespace vex;
competition Competition;

Robot_Telemetry ricky; //Pass data to functions via a global struct named ricky
//seemed efficient i guess

// Autonomousv2
int onauton_autonomous_0() {
    while (Gyroscope.isCalibrating()) { //REALLY IMPORTANT TO CALIBRATE BEFORE MOVING
    vex::task::sleep(50);
  }
  //vex::task Autonoma(AutonomousRoutineDeamon);
  //vex::task::sleep(100);
  //vex::task Vector_Engine(Engine);

  /*while(true){
    printf("%.6f", ricky.CurrentXAxis);
    printf(", ");
    printf("%.6f", ricky.CurrentYAxis);
    printf(", ");
    printf("%.6f", Gyroscope.heading(degrees));
    printf(", Target Velocity: ");
    printf("%.6f", ricky.SetXVelocity);
    printf(", ");
    printf("%.6f", ricky.SetYVelocity);
    printf(", real:");
       printf("%.6f", ricky.DriveXPower);
    printf(", ");
    printf("%.6f", ricky.DriveYPower);
    printf(", converted");
    printf("%.6f", ricky.DriveXPower);
    printf(", ");
    printf("%.6f", ricky.DriveYPower);
        printf(",:: ");
    printf("%.6f", ricky.CurrentThetaValue);
    printf("\n");

    vex::task::sleep(100);
  }*/
return 0;
}

// Driver Control
int ondriver_drivercontrol_0() {
  //onauton_autonomous_0();
  ricky.CurrentXAxis = 230;
  ricky.CurrentYAxis = 220;
  while (true) {
    HeadlessManualDriveTrainControl();
    ManualIntake();
    ManualFlywheel();
    EncoderIntegral();
    ManualEndgame();
    ManualRoller();
    wait(5, msec);
  }
  return 1;
}

// Initalization
int whenStarted1() {
  Gyroscope.startCalibration();
  FL.setStopping(brake);
  RL.setStopping(brake);
  FR.setStopping(brake);
  RR.setStopping(brake);
  RL.setMaxTorque(100.0, percent);
  FR.setMaxTorque(100.0, percent);
  FL.setMaxTorque(100.0, percent);
  RR.setMaxTorque(100.0, percent);
  Flywheel1.setMaxTorque(100.0, percent);
  Flywheel2.setMaxTorque(100.0, percent);
  Flywheel1.setVelocity(0,percent);
  Flywheel2.setVelocity(0,percent);
  Intake.setVelocity(100.0, percent);
  ricky.CurrentXEncoderValue = x.rotation(degrees);
  ricky.CurrentYEncoderValue = y.rotation(degrees);
  EndgameLaunch.set(true);
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


