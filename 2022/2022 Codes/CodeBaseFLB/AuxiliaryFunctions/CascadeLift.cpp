#include "vex.h"

/* This function is used to send the lift to a specific position. */
void SendLiftto(double r){

  //r = r * 1;
  LiftLeft.spinFor(forward, r, degrees, false);
  LiftRight.spinFor(forward, r, degrees, true);

  LiftLeft.stop();
  LiftRight.stop();
}



/* This is the code that is used to control the lift. The code is checking to see if the right trigger
is being pressed. If it is, then it will spin the lift forward. If the lift is already at the
maximum position, then it will stop the lift. */
void ManualLift() {
  extern int LeftInital;
  extern int RightInital;
  if (Controller1.ButtonR1.pressing()) {
    if (LiftLeft.position(degrees) < LeftInital + 1300.0) {
      LiftLeft.spin(forward);
    } else {
      LiftLeft.stop();
    }
    if (LiftRight.position(degrees) < RightInital +  1300.0) {
      LiftRight.spin(forward);
    } else {
      LiftRight.stop();
    }
  }
   else if (Controller1.ButtonL1.pressing()) {
    if (LeftInital - 350.0 < LiftLeft.position(degrees)) {
      LiftLeft.spin(reverse);
    } else {
      LiftLeft.stop();
    }
    if (RightInital - 350.0 < LiftRight.position(degrees)) {
      LiftRight.spin(reverse);
    } else {
      LiftRight.stop();
    }
  } else {
    if (fabs((LiftLeft.position(degrees) - LeftInital) - (LiftRight.position(degrees) - RightInital)) > 10.0) {
      LiftRight.stop();
      LiftLeft.setVelocity(100.0, percent);
      LiftLeft.spinToPosition(LiftRight.position(degrees)-RightInital + LeftInital, degrees, false);
    } 
    else {
      LiftLeft.stop();
      LiftRight.stop();
    }
  }
}