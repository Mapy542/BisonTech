#include "../VectorEngine/Robot_Telemetry_Structure.cpp"
#include "vex.h"

int LocationBasedFlywheelPower() {
  extern Robot_Telemetry ricky;
  float distanceX = fabs(ricky.GoalXPosition - ricky.CurrentXAxis);
  float distanceY = fabs(ricky.CurrentYAxis - ricky.GoalYPosition);
  float distanceD = sqrt(distanceX * distanceX + distanceY * distanceY) *
                    1.3; // calculate distance from goal and add offset for
                         // distance of goal above ground

  int i = 0;
  while (distanceD < ricky.FlywheelPowerDistances[i] ||
         i < 7) { // find maximum power
    i++;
  }
  if (ricky.FlywheelPowerDistances[i] ==
      distanceD) { // if distance is exact return power
    return ricky.FlywheelPowerValues[i];
  }

  float ratio =
      (distanceD - ricky.FlywheelPowerDistances[i - 1]) /
      (ricky.FlywheelPowerDistances[i] -
       ricky.FlywheelPowerDistances[i - 1]); // calculate ratio of distance
                                             // between two points
  return ricky.FlywheelPowerValues[i - 1] +
         ratio *
             (ricky.FlywheelPowerValues[i] -
              ricky.FlywheelPowerValues[i - 1]); // return power based on ratio
}

void ManualFlywheelPID() {
  extern Robot_Telemetry ricky;

  if (Controller1.ButtonB.pressing()) { // spin up flywheel
    ricky.FlywheelTargetVelocity = 75;
  }

  if (Controller1.ButtonY.pressing()) { // spin up flywheel
    ricky.FlywheelTargetVelocity = 55;
  }

  if (Controller1.ButtonX.pressing()) { // spin up flywheel
    ricky.FlywheelTargetVelocity = LocationBasedFlywheelPower(); // 100;
  }

  if (Controller1.ButtonA.pressing()) { // spin down flywheel
    ricky.FlywheelTargetVelocity = 00;
  }

  if (Controller1.ButtonL1.pressing()) {
    Trigger.set(true);
  } else {
    Trigger.set(false);
  }
}

void FlywheelVelocity(int velocity) { // set flywheel velocity
  extern Robot_Telemetry ricky;
  ricky.FlywheelTargetVelocity = velocity;
}

void TriggerPulse(int pulses) { // trigger pulses
  extern Robot_Telemetry ricky;
  // ricky.FlywheelTargetVelocity = LocationBasedFlywheelPower(); // get
  // flywheel
  // power based on
  // distance from
  // goal
  // ricky.DiskCount -= pulses;
  if (pulses == 1) {
    Trigger.set(true);
    vex::task::sleep(750);
    Trigger.set(false);
    return;
  }
  for (int i = 0; i < pulses; i++) {
    while (fabs(Flywheel1.velocity(percent) - ricky.FlywheelTargetVelocity) >
           2) {
      vex::task::sleep(20);
    }
    Trigger.set(true);
    vex::task::sleep(750);
    Trigger.set(false);
    vex::task::sleep(2000);
  }
}

int FlywheelPID() { // flywheel velocity PID
  extern Robot_Telemetry ricky;
  while (true) {
    int Error = ricky.FlywheelTargetVelocity -
                Flywheel1.velocity(percent); // calculate errors

    ricky.FlywheelTotalError += Error; // calculate total error
    if (ricky.FlywheelTotalError > 100) {
      ricky.FlywheelTotalError = 100;
    } else if (ricky.FlywheelTotalError < -100) {
      ricky.FlywheelTotalError = -100;
    }

    int Derivative = Error - ricky.FlywheelLastError; // calculate derivative

    ricky.FlywheelLastError = Error; // set last error

    float Kp, Ki, Kd;                        // set PID constants
    if (ricky.FlywheelTargetVelocity > 70) { // Find tune from LUT
      Kp = ricky.FlywheelHigh[0];
      Ki = ricky.FlywheelHigh[1];
      Kd = ricky.FlywheelHigh[2];
    } else {
      Kp = ricky.FlywheelLow[0];
      Ki = ricky.FlywheelLow[1];
      Kd = ricky.FlywheelLow[2];
    }

    int FlywheelPower = (Kp * Error) + (Ki * ricky.FlywheelTotalError) +
                        (Kd * Derivative); // calculate power
    if (FlywheelPower > 100) {
      FlywheelPower = 100;
    } else if (FlywheelPower < 0) {
      FlywheelPower = 0;
    }

    double voltage =
        (double)FlywheelPower * 12 / 100; // calculate voltage from percentage
    // printf("%.6f", voltage);
    Flywheel1.spin(forward, voltage, volt);
    Flywheel2.spin(forward, voltage, volt);

    vex::task::sleep(5);
  }
}