#include "Robot_Telemetry_Structure.cpp"
#include "vex.h"

// Combines motor powers over directions for multi-axis moving
void DriveMotors(double x, double y, double r, double s) {
  if (!(x == 0.0) || (!(y == 0.0) || !(r == 0.0))) {
    FL.setVelocity((((y * s + r * s) - x * s)), percent);
    FL.spin(forward);
    RL.setVelocity((((y * s + r * s) + x * s)), percent);
    RL.spin(forward);
    FR.setVelocity((((y * s - r * s) + x * s)), percent);
    FR.spin(forward);
    RR.setVelocity((((y * s - r * s) - x * s)), percent);
    RR.spin(forward);
  } else {
    FL.stop();
    RL.stop();
    FR.stop();
    RR.stop();
  }
}

// Does the same thing but take joystick input.
/*void ManualMotors() {
  float ForwardMultiplier = .9;
  float TranslateMultiplier = .5;
  float TurnMultiplier = -.9;
  int CorrectionMultiplier = 0;

  if (!(Controller1.Axis3.position() == 0.0) ||
      (!(Controller1.Axis1.position() == 0.0) ||
       !(Controller1.Axis4.position() == 0.0))) {
    FL.setVelocity((((Controller1.Axis3.position() * ForwardMultiplier +
                      Controller1.Axis4.position() * TranslateMultiplier) -
                     Controller1.Axis1.position() * TurnMultiplier) +
                    CorrectionMultiplier),
                   percent);
    FL.spin(forward);
    RL.setVelocity((((Controller1.Axis3.position() * ForwardMultiplier +
                      Controller1.Axis4.position() * TranslateMultiplier) +
                     Controller1.Axis1.position() * TurnMultiplier) +
                    CorrectionMultiplier),
                   percent);
    RL.spin(forward);
    FR.setVelocity((((Controller1.Axis3.position() * ForwardMultiplier -
                      Controller1.Axis4.position() * TranslateMultiplier) +
                     Controller1.Axis1.position() * TurnMultiplier) -
                    CorrectionMultiplier),
                   percent);
    FR.spin(forward);
    RR.setVelocity((((Controller1.Axis3.position() * ForwardMultiplier -
                      Controller1.Axis4.position() * TranslateMultiplier) -
                     Controller1.Axis1.position() * TurnMultiplier) -
                    CorrectionMultiplier),
                   percent);
    RR.spin(forward);
  } else {
    FL.stop();
    RL.stop();
    FR.stop();
    RR.stop();
  }
}*/

// headless manual control
void ManualDriveTrainControl() {
  extern Robot_Telemetry ricky;
  // extern Robot_Telemetry ricky;

  const float ForwardMultiplier = .8;
  const float TranslateMultiplier = -.7;
  const float TurnMultiplier = .4;

  double X_Speed = Controller1.Axis1.position() * TranslateMultiplier;
  double Y_Speed = Controller1.Axis3.position() * ForwardMultiplier;
  double R_Speed;
  if (!ricky.Override_Manual_R) {
    R_Speed = Controller1.Axis4.position() * TurnMultiplier;
  } else {
    R_Speed = ricky.Override_R_Speed;
  }
  DriveMotors(X_Speed, Y_Speed, R_Speed, 1);
}