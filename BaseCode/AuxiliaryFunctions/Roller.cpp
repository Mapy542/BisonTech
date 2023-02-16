#include "vex.h"

void ManualRoller() {
  if (Controller1.ButtonRight.pressing()) {
    Roller.setVelocity(70, percent);
    Roller.spin(forward);
  } else if (Controller1.ButtonLeft.pressing()) {
    Roller.setVelocity(70, percent);
    Roller.spin(reverse);
  } else {
    Roller.stop();
  }
}