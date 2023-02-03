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

  /*if (Controller1.ButtonX.pressing()) { // reverse flywheel to unstick
    while (Controller1.ButtonX.pressing()) {
      Flywheel1.spin(reverse);
      Flywheel2.spin(reverse);

      Flywheel1.setVelocity(20, percent);
      Flywheel1.setVelocity(20, percent);
    }

    Flywheel1.setVelocity(0, percent);
    Flywheel2.setVelocity(0, percent);

    Flywheel1.spin(forward);
    Flywheel2.spin(forward);
  }*/

  if (Controller1.ButtonL1.pressing()) {
    Trigger.set(true);
  } else {
    Trigger.set(false);
  }
}

void FlywheelVelocity(int velocity) { // set flywheel velocity
  extern Robot_Telemetry ricky;
  ricky.SetFlywheelSpeed = velocity;
  Flywheel1.setVelocity(velocity, percent);
  Flywheel2.setVelocity(velocity, percent);
  Flywheel1.spin(forward);
  Flywheel2.spin(forward);
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
    while (fabs(Flywheel1.velocity(percent) - ricky.SetFlywheelSpeed) > 2) {
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