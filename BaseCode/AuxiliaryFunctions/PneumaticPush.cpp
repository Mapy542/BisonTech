#include "vex.h"

/* This is a function that will make the piston go up or down depending on
whether the Y button is pressed. */
void ManualPiston() {
  if (Controller1.ButtonY.pressing()) {
    DigitalOutH.set(true);
  } else {
    DigitalOutH.set(false);
  }
}

/* This function will make the rod go up and down twice. */
void RodKnock() {
  DigitalOutH.set(true);
  wait(0.15, seconds);
  DigitalOutH.set(false);
  wait(0.15, seconds);

  DigitalOutH.set(true);
  wait(0.15, seconds);
  DigitalOutH.set(false);
}