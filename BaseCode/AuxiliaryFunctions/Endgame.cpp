#include "vex.h"

void ManualEndgame() {
  if (Controller1.ButtonUp.pressing()) {
    EndgameLaunch.set(false);
  } else {
    EndgameLaunch.set(true);
  }
}
