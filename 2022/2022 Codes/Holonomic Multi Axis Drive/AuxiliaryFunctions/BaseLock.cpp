#include "vex.h"

void Lock_Base() {
  BaseLock.spinToPosition(45.0, degrees, false);
}


void Unlock_Base() {
  BaseLock.spinToPosition(0.0, degrees, false);
}

void ManualLockToggle() {
  extern int LockDesiredState;

  if (Controller1.ButtonA.pressing()) {
    if (LockDesiredState > 30.0) {
      LockDesiredState = 0.0;
      Unlock_Base();
    }
    else {
      LockDesiredState = 35.0;
      Lock_Base();
    }
    waitUntil((!Controller1.ButtonA.pressing()));
  }
}