#include "vex.h"

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
  float ForwardMultiplier = .9;
  float TranslateMultiplier = .5;
  float TurnMultiplier = -.9;
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

//Drives motors with a nonchaning heading.
void HeadlessManualDrive() {
  float ForwardMultiplier = .9;
  float TranslateMultiplier = -.9;
  float TurnMultiplier = .7;
  
  float InterpolatedForward = (Controller1.Axis3.position() * cos(Gyroscope.heading(degrees))) + (Controller1.Axis4.position() * sin(Gyroscope.heading(degrees)));
  float InterpolatedTranslation = (Controller1.Axis3.position() * sin(Gyroscope.heading(degrees))) + (Controller1.Axis4.position() * cos(Gyroscope.heading(degrees)));
  float InterpolatedRotation = Controller1.Axis1.position();

  if (!(InterpolatedForward == 0.0) || (!(InterpolatedTranslation == 0.0) || !(InterpolatedRotation == 0.0))) {
    FL.setVelocity((((InterpolatedForward * ForwardMultiplier + InterpolatedRotation * TurnMultiplier) - InterpolatedTranslation * TranslateMultiplier) + CorrectionMultiplier), percent);
    FL.spin(forward);
    RL.setVelocity((((InterpolatedForward * ForwardMultiplier + InterpolatedRotation * TurnMultiplier) + InterpolatedTranslation * TranslateMultiplier) + CorrectionMultiplier), percent);
    RL.spin(forward);
    FR.setVelocity((((InterpolatedForward * ForwardMultiplier - InterpolatedRotation * TurnMultiplier) + InterpolatedTranslation * TranslateMultiplier) - CorrectionMultiplier), percent);
    FR.spin(forward);
    RR.setVelocity((((InterpolatedForward * ForwardMultiplier - InterpolatedRotation * TurnMultiplier) - InterpolatedTranslation * TranslateMultiplier) - CorrectionMultiplier), percent);
    RR.spin(forward);
  }
  else {
    FL.stop();
    RL.stop();
    FR.stop();
    RR.stop();
  }
}