#include "vex.h"

/* Spinning the base lock to a position of 45 degrees. */
void Extend_Back() { BackMotor.spinToPosition(200.0, degrees, false); }

/* This function spins the base lock to a position of 0 degrees. */
void Retract_Back() { BackMotor.spinToPosition(0.0, degrees, false); }

/* This function is used to toggle the lock between the locked and unlocked
 * states. */
void ManualBack() {
  extern int BackDesiredState;

  if (Controller1.ButtonX.pressing()) {
    if (BackDesiredState > 30.0) {
      BackDesiredState = 0.0;
      Extend_Back();
    } else {
      BackDesiredState = 35.0;
      Retract_Back();
    }
    waitUntil((!Controller1.ButtonX.pressing()));
  }
}

void CloseBack() { DigitalOutG.set(true); }

void OpenBack() { DigitalOutG.set(false); }

void ManualBackGripper() {
  extern int BackGripperDesiredState;

  if (Controller1.ButtonB.pressing()) {
    if (BackGripperDesiredState > 30.0) {
      BackGripperDesiredState = 0.0;
      CloseBack();
    } else {
      BackGripperDesiredState = 35.0;
      OpenBack();
    }
    waitUntil((!Controller1.ButtonB.pressing()));
  }
}