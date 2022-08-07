#include "vex.h"

// find the lowest angle of the base lock motor to ensure there is no back lash
void DynamicBoundsDetection() {
  extern int BaseLockOffset;
  BaseLock.setVelocity(30.0, percent);
  BaseLock.spinToPosition(500, degrees, false);
  wait(200, msec);
  while (BaseLock.velocity(percent) > .3) {

    wait(100, msec);
  }
  BaseLockOffset = BaseLock.position(degrees);
  BaseLock.stop();
}

/* Spinning the base lock to a position of 45 degrees. */
void Lock_Base() {
  extern int BaseLockOffset;
  BaseLock.spinToPosition(BaseLockOffset, degrees, false);
}

/* This function spins the base lock to a position of 0 degrees. */
void Unlock_Base() {
  extern int BaseLockOffset;
  BaseLock.spinToPosition(BaseLockOffset - 55, degrees, false);
}

/* This function is used to toggle the lock between the locked and unlocked
 * states. */
void ManualLockToggle() {
  extern int LockDesiredState;

  if (Controller1.ButtonA.pressing()) {
    if (LockDesiredState == -99) {
      DynamicBoundsDetection();
      LockDesiredState = 0;
    } else {
      if (LockDesiredState > 0.0) {
        LockDesiredState = 0.0;
        Unlock_Base();
      } else {
        LockDesiredState = 1.0;
        Lock_Base();
      }
    }
    waitUntil((!Controller1.ButtonA.pressing()));
  }
}
