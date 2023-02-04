#include "../VectorEngine/Robot_Telemetry_Structure.cpp"
#include "vex.h"

/*int LocationBasedFlywheelPower() {
  extern Robot_Telemetry ricky;
  float distance = fabs(fabs(ricky.GoalXPosition) - fabs(ricky.CurrentXAxis)) +
                   fabs(ricky.CurrentYAxis - ricky.GoalYPosition);
  double power = (double)distance / ricky.MaximumFlywheelDistance;
  if (power < ricky.FlywheelMin) {
    power = ricky.FlywheelMin;
  }
  if (power > ricky.FlywheelMax) {
    power = ricky.FlywheelMax;
  }
  return (int)(power * 100);
}*/

void ManualFlywheel() {
  extern Robot_Telemetry ricky;

  // int power = LocationBasedFlywheelPower(); // get flywheel power based on
  // distance from goal

  if (Controller1.ButtonB.pressing()) { // spin up flywheel
    Flywheel1.setVelocity(80, percent); // power, percent);
    Flywheel2.setVelocity(80, percent); // power, percent);
  }

  if (Controller1.ButtonY.pressing()) { // spin up flywheel
    Flywheel1.setVelocity(50, percent); // power, percent);
    Flywheel2.setVelocity(50, percent); // power, percent);
  }
  if (Controller1.ButtonX.pressing()) {  // spin up flywheel
    Flywheel1.setVelocity(100, percent); // power, percent);
    Flywheel2.setVelocity(100, percent); // power, percent);
  }

  if (Controller1.ButtonA.pressing()) { // spin down flywheel
    Flywheel1.setVelocity(0, percent);
    Flywheel2.setVelocity(0, percent);
  }
  Flywheel1.spin(forward);
  Flywheel2.spin(forward);

  if (Controller1.ButtonL1.pressing()) {
    Trigger.set(true);
  } else {
    Trigger.set(false);
  }
}

void ManualFlywheelPID() {
  extern Robot_Telemetry ricky;

  // int power = LocationBasedFlywheelPower(); // get flywheel power based on
  // distance from goal

  if (Controller1.ButtonB.pressing()) { // spin up flywheel
    ricky.FlywheelTargetVelocity = 80;
  }

  if (Controller1.ButtonY.pressing()) { // spin up flywheel
    ricky.FlywheelTargetVelocity = 50;
  }
  if (Controller1.ButtonX.pressing()) { // spin up flywheel
    ricky.FlywheelTargetVelocity = 100;
  }

  if (Controller1.ButtonA.pressing()) { // spin down flywheel
    ricky.FlywheelTargetVelocity = 00;
  }

  if (Controller1.ButtonL1.pressing()) {
    Trigger.set(true);
  } else {
    Trigger.set(false);
  }
}

void FlywheelVelocity(int velocity) { // set flywheel velocity
  extern Robot_Telemetry ricky;
  ricky.FlywheelTargetVelocity = velocity;
}

void TriggerPulse(int pulses) { // trigger pulses
  extern Robot_Telemetry ricky;
  if (pulses == 1) {
    Trigger.set(true);
    vex::task::sleep(750);
    Trigger.set(false);
    return;
  }
  for (int i = 0; i < pulses; i++) {
    while (fabs(Flywheel1.velocity(percent) - ricky.FlywheelTargetVelocity) >
           2) {
      Flywheel1.spin(forward);
      Flywheel2.spin(forward);
      vex::task::sleep(20);
    }
    Trigger.set(true);
    vex::task::sleep(750);
    Trigger.set(false);
    vex::task::sleep(2000);
  }
}

int FlywheelPID() { // flywheel velocity PID
  extern Robot_Telemetry ricky;
  while (true) {
    int Error = ricky.FlywheelTargetVelocity -
                Flywheel1.velocity(percent); // calculate errors

    ricky.FlywheelTotalError += Error; // calculate total error
    if (ricky.FlywheelTotalError > 100) {
      ricky.FlywheelTotalError = 100;
    } else if (ricky.FlywheelTotalError < -100) {
      ricky.FlywheelTotalError = -100;
    }

    int Derivative = Error - ricky.FlywheelLastError; // calculate derivative

    ricky.FlywheelLastError = Error; // set last error

    int FlywheelPower = (ricky.FlywheelKp * Error) +
                        (ricky.FlywheelKi * ricky.FlywheelTotalError) +
                        (ricky.FlywheelKd * Derivative); // calculate power
    if (FlywheelPower > 100) {
      FlywheelPower = 100;
    } else if (FlywheelPower < 0) {
      FlywheelPower = 0;
    }

    double voltage =
        (double)FlywheelPower * 12 / 100; // calculate voltage from percentage
    // printf("%.6f", voltage);
    Flywheel1.spin(forward, voltage, volt);
    Flywheel2.spin(forward, voltage, volt);

    vex::task::sleep(5);
  }
}