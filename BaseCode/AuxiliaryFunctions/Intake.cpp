#include "vex.h"

void ManualIntake() {
  if (Controller1.ButtonR1.pressing()) {
    Intake.setVelocity(100, percent);
  } else if (Controller1.ButtonR2.pressing()) {
    Intake.setVelocity(-100, percent);
  } else {
    Intake.setVelocity(0, percent);
  }
  Intake.spin(forward);
}

void IntakeVelocity(int velocity) {
  Intake.setVelocity(velocity, percent);
  Intake.spin(forward);
}