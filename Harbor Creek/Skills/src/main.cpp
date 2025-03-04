#include "C:\Users\elimb\Documents\GitHub\BisonTech\BaseCode\AutonomousCodes.cpp"
#include "vex.h"


using namespace vex;
competition Competition;

Robot_Telemetry ricky; // Pass data to functions via a global struct named ricky
// seemed efficient i guess

// Autonomousv2
int onauton_autonomous_0() {
  ricky.TravelImpeded = false;
  while (Gyroscope
             .isCalibrating()) { // REALLY IMPORTANT TO CALIBRATE BEFORE MOVING
    vex::task::sleep(50);
  }
  vex::task Autonoma(AutonomousRoutineDeamon);
  vex::task::sleep(100);
  vex::task Vector_Engine(Engine);
  vex::task::sleep(100);
  vex::task FlywheelControl(FlywheelPID);
  vex::task DiskDetector(VisionDeamon);
  vex::task Stucky(Preliminary_Lock_Detect);

 while(true){
    printf("%.6f",ricky.CurrentXAxis);
    printf(", ");
    printf("%.6f", ricky.CurrentYAxis);
    printf(", ");
    printf("%.6f", Gyroscope.heading(degrees));
    printf(", Target Velocity: ");
    printf("%.6f", ricky.TargetXVelocity);
    printf(", ");
    printf("%.6f", ricky.TargetYVelocity);
    printf(", ramp:");
    printf("%.6f", ricky.TargetTotalVelocity);
    printf("\n");
    if(ricky.AutoDone){
      Autonoma.stop();
      Vector_Engine.stop();
      FlywheelControl.stop();
      DiskDetector.stop();
      Stucky.stop();
    }
    vex::task::sleep(50);
  }
return 0;
}

// Driver Control
int ondriver_drivercontrol_0() {
  //onauton_autonomous_0();
  Brain.resetTimer();
    while (Gyroscope
             .isCalibrating()) { // REALLY IMPORTANT TO CALIBRATE BEFORE MOVING
    vex::task::sleep(50);
  }
  vex::task FlywheelControl(FlywheelPID);
  //Gyroscope.setHeading(180, degrees);
  //ricky.CurrentXAxis = 230;
  //ricky.CurrentYAxis = 220;
  vex::task Engine(DriverSupplementEngine);
  vex::task DiskDetector(VisionDeamon);
  while (true) {
    ManualDriveTrainControl();
    ManualIntake();
    ManualFlywheelPID();
    ManualEndgame();
    ManualRoller();
        printf("%.6f", (fabs(ricky.CurrentXVelocity) + fabs(ricky.CurrentYVelocity) +
        fabs(ricky.CurrentRVelocity)) *
           100);
        printf("\n");
  }
  return 1;
}

// Initalization
int whenStarted1() {
  Gyroscope.startCalibration();
  FL.setStopping(coast);
  RL.setStopping(coast);
  FR.setStopping(coast);
  RR.setStopping(coast);
  RL.setMaxTorque(75.0, percent);
  FR.setMaxTorque(75.0, percent);
  FL.setMaxTorque(100.0, percent);
  RR.setMaxTorque(100.0, percent);
  Flywheel1.setMaxTorque(100.0, percent);
  Flywheel2.setMaxTorque(100.0, percent);
  Flywheel1.setVelocity(0, percent);
  Flywheel2.setVelocity(0, percent);
  Intake.setVelocity(100.0, percent);
  ricky.CurrentXEncoderValue = x.rotation(degrees);
  ricky.CurrentYEncoderValue = y.rotation(degrees);
  EndgameLaunch.set(true);
  return 0;
}

void VEXcode_driver_task() {
  // Start the driver control tasks....
  vex::task drive0(ondriver_drivercontrol_0);
  while ((Competition.isDriverControl() && Competition.isEnabled())) {
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
