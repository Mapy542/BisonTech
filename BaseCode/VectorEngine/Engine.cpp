#include "MotorDriver.cpp"
#include "Odometry.cpp"
#include "vex.h"
#include <cmath>

// RadialDistance given theta1 is the gyro reading
float FromGyro(double r) {
  double DegreeDiff = 0;
  if (fabs((Gyroscope.heading(degrees) - r)) > 180.0) {
    DegreeDiff = ((Gyroscope.heading(degrees) - r) - 359.0) * -1.0;
  } else {
    DegreeDiff = Gyroscope.heading(degrees) - r;
  }
  return DegreeDiff;
}

void ProximityRamp() {
  extern Robot_Telemetry ricky;
  double RampDown =
      ((fabs(ricky.TargetXAxis - ricky.CurrentXAxis) +
        fabs(ricky.TargetYAxis - ricky.CurrentYAxis)) +
       fabs(FromGyro(ricky.TargetTheta) * -3.14159265359 * 370 / 180.0)) *
      0.15; // find total distance to target
  RampDown = (RampDown * RampDown) / 6000.0 +
             0.05; // i have no idea how this works at this point. no matter
                   // how i change it it doesn't improve ramp performance
                   // just over shoots.
  if (RampDown > 1.0) { // if rampdown is greater than 1.0 then limit it to
                        // 1.0 to prevent overdrive;
    RampDown = 1.0;
  }
  ricky.TargetTotalVelocity =
      ricky.TargetSpeed * RampDown; // set target
                                    // velocity to
                                    // target speed affected by rampdown
}

void MotorVectorEngine() { // main calculation loop
  extern Robot_Telemetry ricky;
  if (ricky.SetXVelocity != 0 || ricky.SetYVelocity != 0 ||
      ricky.SetRVelocity != 0) { // if the robot is moving then update busy flag
                                 // for further scripting
    ricky.EngineBusy = true;
  } else {
    ricky.EngineBusy = false;
  }
  double DeltaXVelocity =
      (ricky.TargetXVelocity -
       ricky.SetXVelocity); // find change in velocity percent
  if (DeltaXVelocity >
      ricky.MaxAcceleration) { // if change is greater than max acceleration
    DeltaXVelocity = ricky.MaxAcceleration; // limit change to max acceleration
  } else if (DeltaXVelocity < -ricky.MaxAcceleration) {
    DeltaXVelocity = -ricky.MaxAcceleration;
  }
  ricky.SetXVelocity += DeltaXVelocity;
  double DeltaYVelocity = (ricky.TargetYVelocity - ricky.SetYVelocity);
  if (DeltaYVelocity > ricky.MaxAcceleration) {
    DeltaYVelocity = ricky.MaxAcceleration;
  } else if (DeltaYVelocity < -ricky.MaxAcceleration) {
    DeltaYVelocity = -ricky.MaxAcceleration;
  }
  ricky.SetYVelocity += DeltaYVelocity;
  double DeltaRVelocity = (ricky.TargetRVelocity - ricky.SetRVelocity);
  if (DeltaRVelocity > ricky.MaxAcceleration) {
    DeltaRVelocity = ricky.MaxAcceleration;
  } else if (DeltaRVelocity < -ricky.MaxAcceleration) {
    DeltaRVelocity = -ricky.MaxAcceleration;
  }
  ricky.SetRVelocity += DeltaRVelocity;

  // convert to polar coordinates and rotate by difference of starting point and
  // current theta
  PolarTransformation(ricky.SetXVelocity, ricky.SetYVelocity,
                      180 - ricky.CurrentThetaValue);

  DriveMotors(ricky.TransformReturnX, ricky.TransformReturnY,
              ricky.SetRVelocity,
              ricky.TargetTotalVelocity); // send to motor driver
}

void Direct_Vector_Generator() { // calculate direct vector to target coords
  extern Robot_Telemetry ricky;
  if (fabs(ricky.TargetXAxis - ricky.CurrentXAxis) < ricky.DistanceTolerance &&
      fabs(ricky.TargetYAxis - ricky.CurrentYAxis) < ricky.DistanceTolerance &&
      fabs(ricky.TargetTheta - ricky.CurrentThetaValue) <
          ricky.AngleTolerance) { // within tolerance stop robot to prevent over
                                  // tune
    ricky.TargetXVelocity = 0;
    ricky.TargetYVelocity = 0;
    ricky.TargetRVelocity = 0; // set all velocities to 0 to stop robot
  } else {
    double TangentialDist =
        FromGyro(ricky.TargetTheta) * -3.14159265359 * 370 /
        180.0; // find tangential distance of wheels based on radius and heading
    if ((fabs(ricky.TargetXAxis - ricky.CurrentXAxis) < fabs(TangentialDist) &&
         fabs(ricky.TargetYAxis - ricky.CurrentYAxis) < fabs(TangentialDist)) ||
        ricky.Targeting) { // if tangential distance is more than x
                           // and y distance or targeting is enabled

      ricky.TargetYVelocity =
          100.0 * ((ricky.TargetYAxis - ricky.CurrentYAxis) /
                   fabs((TangentialDist))); // scale y to y over tangential
      ricky.TargetXVelocity =
          100.0 * ((ricky.TargetXAxis - ricky.CurrentXAxis) /
                   fabs((TangentialDist))); // scale x to x over tangential
      ricky.TargetRVelocity =
          copysign(50.0, TangentialDist); // scale r to 50% as r is 200% more
                                          // powerful than x and y.

    } else if (fabs((ricky.TargetXAxis - ricky.CurrentXAxis)) <
               fabs((ricky.TargetYAxis -
                     ricky.CurrentYAxis))) { // scale given x is the limiting
                                             // factor

      ricky.TargetYVelocity =
          copysign(100.0, (ricky.TargetYAxis - ricky.CurrentYAxis));
      ricky.TargetXVelocity =
          100.0 * ((ricky.TargetXAxis - ricky.CurrentXAxis) /
                   fabs((ricky.TargetYAxis - ricky.CurrentYAxis)));
      ricky.TargetRVelocity =
          50.0 *
          ((TangentialDist) / fabs((ricky.TargetYAxis - ricky.CurrentYAxis)));
    } else { // scale given y is the limiting factor

      ricky.TargetYVelocity =
          100.0 * ((ricky.TargetYAxis - ricky.CurrentYAxis) /
                   fabs((ricky.TargetXAxis - ricky.CurrentXAxis)));
      ricky.TargetXVelocity =
          copysign(100.0, (ricky.TargetXAxis - ricky.CurrentXAxis));
      ricky.TargetRVelocity =
          50.0 *
          ((TangentialDist) / fabs((ricky.TargetXAxis - ricky.CurrentXAxis)));
    }
    ProximityRamp(); // ramp to target velocity
    // limit velocities
    if (fabs(ricky.TargetXVelocity) > 100) {
      ricky.TargetXVelocity = 100;
    }
    if (fabs(ricky.TargetYVelocity) > 100) {
      ricky.TargetYVelocity = 100;
    }
    if (fabs(ricky.TargetRVelocity) > 100) {
      ricky.TargetRVelocity = 100;
    }
  }
}

int Engine() { // Main engine loop
  while (true) {
    EncoderIntegral();         // Get odometry from encoders
    Direct_Vector_Generator(); // Calculate motor powers from inputs in
                               // Robot_Telemetry structure
    MotorVectorEngine();       // Calculate motor powers from inputs in
    vex::task::sleep(5);
  }
};
/*
double RotationalRamp() {
  extern Robot_Telemetry ricky;
  double RampDown =
      (fabs(FromGyro(ricky.TargetTheta) * -3.14159265359 * 370 / 180.0)) *
      0.15; // find total distance to target
  RampDown = (RampDown * RampDown) / 6000.0 +
             0.05; // i have no idea how this works at this point. no matter
                   // how i change it it doesn't improve ramp performance
                   // just over shoots.
  if (RampDown > 1.0) { // if rampdown is greater than 1.0 then limit it to
                        // 1.0 to prevent overdrive;
    RampDown = 1.0;
  }
  return RampDown;
}

void FlywheelRotationEngine() { // main calculation loop
  extern Robot_Telemetry ricky;
  if (ricky.SetRVelocity != 0) { // if the robot is moving then update busy flag
                                 // for further scripting
    ricky.EngineBusy = true;
  } else {
    ricky.EngineBusy = false;
  }
  double DeltaRVelocity = (ricky.TargetRVelocity - ricky.SetRVelocity);
  if (DeltaRVelocity > ricky.MaxAcceleration) {
    DeltaRVelocity = ricky.MaxAcceleration;
  } else if (DeltaRVelocity < -ricky.MaxAcceleration) {
    DeltaRVelocity = -ricky.MaxAcceleration;
  }
  ricky.SetRVelocity += DeltaRVelocity;
  ricky.Override_R_Speed = ricky.SetRVelocity;
}

void Rotational_Vector_Generator() { // calculate direct vector to target coords
  extern Robot_Telemetry ricky;
  if (fabs(ricky.TargetTheta - ricky.CurrentThetaValue) <
      ricky.AngleTolerance) { // within tolerance stop robot to prevent over
                              // tune

    ricky.TargetRVelocity = 0; // set all velocities to 0 to stop robot
  } else {
    double TangentialDist =
        FromGyro(ricky.TargetTheta) * -3.14159265359 * 370 /
        180.0; // find tangential distance of wheels based on radius and heading
    ricky.TargetRVelocity =
        copysign(50.0, TangentialDist); // scale r to 50% as r is 200% more
                                        // powerful than x and y.

    ricky.TargetRVelocity *= RotationalRamp(); // ramp to target velocity
    // limit velocities

    if (fabs(ricky.TargetRVelocity) > 100) {
      ricky.TargetRVelocity = 100;
    }
  }
}

void FlywheelAutoRotate() {
  extern Robot_Telemetry ricky;

  double X = ricky.GoalXPosition - ricky.CurrentXAxis;
  double Y = ricky.CurrentYAxis - ricky.GoalYPosition;
  double Angle = ToPolarAngle(X, Y) - 90 + ricky.LauncherAngleCompensation;
  ricky.TargetTheta = Angle;
}

int DriverSupplementEngine() {
  extern Robot_Telemetry ricky;
  while (true) {
    EncoderIntegral(); // Get odometry from encoders
    if (Controller1.ButtonL2.pressing()) {
      ricky.Override_Manual_R = true;
      FlywheelAutoRotate();
      Rotational_Vector_Generator();
      FlywheelRotationEngine();
    } else {
      ricky.Override_Manual_R = false;
    }
    vex::task::sleep(5);
  }
}
*/
