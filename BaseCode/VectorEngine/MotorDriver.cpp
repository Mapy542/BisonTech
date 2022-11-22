#include "vex.h"

// Combines motor powers over directions for multiaxis movings
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
  } else {
    FL.stop();
    RL.stop();
    FR.stop();
    RR.stop();
  }
}

// Does the same thing but take joystick input.
void ManualMotors() {
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
}

// post condition: return the result of the log function in desmos where x is
// the joystick value note: log only works with positive values check if the
// value is positive or negative and use the first or second log function
int JoystickLog(int joystickvalue) {
  int output = 0;
  if (joystickvalue > 0) {
    output = log(joystickvalue + 1) * 50;
  } else if (joystickvalue < 0) {
    output = log(-1 * joystickvalue + 1) * -50;
  }
  return output;
}

// post condition: return the result of the x^2.2 function in desmos where x is
// the joystick value
int JoystickCubic(int joystickvalue) { return pow(joystickvalue, 2.2) / 200; }

// headless manual control
void HeadlessManualDriveTrainControl() {
  extern struct Robot_Telemetry ricky;
  // extern Robot_Telemetry ricky;

  // x component is the horizontal movement of the joystick times the cosine of
  // the gyro angle times pi over 180. the pi and 180 convert the ouput of sin
  // and cos from degrees to radians i assume.
  double X_Speed = JoystickCubic(Controller1.Axis3.position()) *
                       sin(Gyroscope.heading(degrees)) * M_PI / 180 +
                   JoystickCubic(Controller1.Axis4.position()) *
                       cos(Gyroscope.heading(degrees)) * M_PI / 180;
  double Y_Speed = JoystickCubic(Controller1.Axis3.position()) *
                       cos(Gyroscope.heading(degrees)) * M_PI / 180 +
                   JoystickCubic(Controller1.Axis4.position()) *
                       sin(Gyroscope.heading(degrees)) * M_PI / 180;
  double R_Speed;
  if (!ricky.Overide_Manual_R) {
    R_Speed = Controller1.Axis1.position();
  } else {
    R_Speed = ricky.Override_R_Speed;
  }
  DriveMotors(X_Speed, Y_Speed, R_Speed, 1);
}