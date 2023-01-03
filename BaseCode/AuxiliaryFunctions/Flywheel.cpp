#include "vex.h"

void ManualFlywheel() {
  if (Controller1.ButtonB.pressing()) {
    Flywheel1.setVelocity(100, percent);
    Flywheel2.setVelocity(100, percent);
  }

  if (Controller1.ButtonA.pressing()) {
    Flywheel1.setVelocity(0, percent);
    Flywheel2.setVelocity(0, percent);
  }
  Flywheel1.spin(forward);
  Flywheel2.spin(forward);
}