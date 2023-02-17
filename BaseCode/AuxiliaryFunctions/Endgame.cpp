#include "vex.h"

void ManualEndgame() {
  if (Controller1.ButtonUp.pressing() &&
      Brain.timer(msec) > 96000) { // allows for 9 seconds of endgame
    EndgameLaunch.set(false);
  } else {
    EndgameLaunch.set(true);
  }
}