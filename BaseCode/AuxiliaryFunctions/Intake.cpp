#include "vex.h"

void ManualIntake() {
  if (Controller1.ButtonR1.pressing()) {
    Intake.setVelocity(70, percent);
  } else if (Controller1.ButtonR2.pressing()) {
    Intake.setVelocity(-70, percent);
  } else {
    Intake.setVelocity(0, percent);
  }
  Intake.spin(forward);
}