#include "vex.h"

//lmao sigma balls

//Combines motor powers over directions for multiaxis movings
void DriveMotors(double x, double y, double r, double s) {
  if (!(x == 0.0) || (!(y == 0.0) || !(r == 0.0))) {
    FL.setVelocity((((y * s + r * s) - x * s) + 0.0), percent);
    FL.spin(forward);
    RL.setVelocity((((y * s + r * s) + x * s) + 0.0), percent);
    RL.spin(forward);
    FR.setVelocity((((y * s - r * s) + x * s) - 0.0), percent);
    FR.spin(forward);
    RR.setVelocity((((y * s - r * s) - x * s) - 0.0), percent);
    RR.spin(forward);
  }
  else {
    FL.stop();
    RL.stop();
    FR.stop();
    RR.stop();
  }
}

//Does the same thing but take joystick input.
void ManualMotors() {
  float ForwardMultiplier = .7;
  float TranslateMultiplier = .4;
  float TurnMultiplier = -.7;
  int CorrectionMultiplier = 0;

  if (!(Controller1.Axis3.position() == 0.0) || (!(Controller1.Axis1.position() == 0.0) || !(Controller1.Axis4.position() == 0.0))) {
    FL.setVelocity((((Controller1.Axis3.position() * ForwardMultiplier + Controller1.Axis4.position() * TranslateMultiplier) - Controller1.Axis1.position() * TurnMultiplier) + CorrectionMultiplier), percent);
    FL.spin(forward);
    RL.setVelocity((((Controller1.Axis3.position() * ForwardMultiplier + Controller1.Axis4.position() * TranslateMultiplier) + Controller1.Axis1.position() * TurnMultiplier) + CorrectionMultiplier), percent);
    RL.spin(forward);
    FR.setVelocity((((Controller1.Axis3.position() * ForwardMultiplier - Controller1.Axis4.position() * TranslateMultiplier) + Controller1.Axis1.position() * TurnMultiplier) - CorrectionMultiplier), percent);
    FR.spin(forward);
    RR.setVelocity((((Controller1.Axis3.position() * ForwardMultiplier - Controller1.Axis4.position() * TranslateMultiplier) - Controller1.Axis1.position() * TurnMultiplier) - CorrectionMultiplier), percent);
    RR.spin(forward);
  }
  else {
    FL.stop();
    RL.stop();
    FR.stop();
    RR.stop();
  }
}