#include "Robot_Telemetry_Structure.cpp"
#include "vex.h"

// Combines motor powers over directions for multi-axis moving
void DriveMotors(double x, double y, double r, double s) {
  if (!(x == 0.0) || (!(y == 0.0) || !(r == 0.0))) {
    FL.setVelocity((((y * s + r * s) - x * s)) * 1.1, percent);
    FL.spin(forward);
    RL.setVelocity((((y * s + r * s) + x * s)), percent);
    RL.spin(forward);
    FR.setVelocity((((y * s - r * s) + x * s)), percent);
    FR.spin(forward);
    RR.setVelocity((((y * s - r * s) - x * s)) * 1.1, percent);
    RR.spin(forward);
  } else {
    FL.stop();
    RL.stop();
    FR.stop();
    RR.stop();
  }
}

// manual control
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
    DriveMotors(X_Speed, Y_Speed, R_Speed, 1);
  } else {
    vex::task::sleep(50);
  }
}