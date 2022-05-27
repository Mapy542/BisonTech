#include "vex.h"

//Used for autonomous
void SendLiftto(double r) {
  LiftLeft.spinToPosition(r, degrees, false);
  LiftRight.spinToPosition(r, degrees, true);
}

//Used for driver control
void ManualLift() {
  if (Controller1.ButtonR1.pressing()) {
    if (LiftLeft.rotation(degrees) < 2000.0) {
      LiftLeft.spin(forward);
    } else {
      LiftLeft.stop();
    }
    if (LiftRight.rotation(degrees) < 2000.0) {
      LiftRight.spin(forward);
    } else {
      LiftRight.stop();
    }
  } else if (Controller1.ButtonL1.pressing()) {
    if (-600.0 < LiftLeft.rotation(degrees)) {
      LiftLeft.spin(reverse);
    } else {
      LiftLeft.stop();
    }
    if (-600.0 < LiftRight.rotation(degrees)) {
      LiftRight.spin(reverse);
    } else {
      LiftRight.stop();
    }
  } else {
    LiftLeft.stop();
    LiftRight.stop();
    if (LiftLeft.rotation(degrees) < LiftRight.rotation(degrees)) {
      LiftRight.stop();
      LiftLeft.setVelocity(100.0, percent);
      LiftLeft.spinToPosition(LiftRight.rotation(degrees), degrees, false);
    } else {
      LiftLeft.stop();
      LiftRight.setVelocity(100.0, percent);
      LiftRight.spinToPosition(LiftLeft.rotation(degrees), degrees, false);
    }
  }
}