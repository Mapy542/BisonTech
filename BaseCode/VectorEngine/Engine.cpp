#include "MotorDriver.cpp"
#include "Odometry.cpp"
#include "vex.h"
#include <cmath>

// RadialDistance given theta1 is the gyro reading
float FromGyro(double r) {
  double DegreeDiff = 0;
  if (fabs(Gyroscope.heading(degrees) - r) > 180.0) {
    DegreeDiff = ((Gyroscope.heading(degrees) - r) - 359.0) * -1.0;
  } else {
    DegreeDiff = Gyroscope.heading(degrees) - r;
  }
  return DegreeDiff;
}

void ProximityRamp() { // ramp down to target speed based on distance to target
  extern Robot_Telemetry ricky;
  double RampDown =
      ((fabs(ricky.TargetXAxis - ricky.CurrentXAxis) +
        fabs(ricky.TargetYAxis - ricky.CurrentYAxis)) +
       fabs(FromGyro(ricky.TargetTheta) * -3.14159265359 * 370 / 180.0)) *
      0.15; // find total distance to target
  RampDown = (RampDown * RampDown) / 3000.0 +
             0.1; // i have no idea how this works at this point. no matter
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

  // convert to polar coordinates and rotate by difference of starting point and
  // current theta
  EnginePolarTransformation(ricky.TargetXVelocity, ricky.TargetYVelocity,
                            180 - ricky.CurrentThetaValue);

  DriveMotors(ricky.EngineTransformReturnX, ricky.EngineTransformReturnY,
              ricky.TargetRVelocity,
              ricky.TargetTotalVelocity); // send to motor driver
}

void Direct_Vector_Generator() { // calculate direct vector to target coords
  extern Robot_Telemetry ricky;
  if (fabs(ricky.TargetXAxis - ricky.CurrentXAxis) < ricky.DistanceTolerance &&
      fabs(ricky.TargetYAxis - ricky.CurrentYAxis) < ricky.DistanceTolerance &&
      fabs(ricky.TargetTheta - ricky.CurrentThetaValue) <
          ricky.AngleTolerance) { // within tolerance stop robot to prevent
                                  // over tune
    ricky.TargetXVelocity = 0;
    ricky.TargetYVelocity = 0;
    ricky.TargetRVelocity = 0; // set all velocities to 0 to stop robot
    ricky.EngineBusy = false;  // set engine to not busy
  } else {
    ricky.EngineBusy = true; // set engine to busy
    double TangentialDist = FromGyro(ricky.TargetTheta) * -3.14159265359 * 370 /
                            180.0; // find tangential distance of wheels
                                   // based on radius and heading
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
                     ricky.CurrentYAxis))) { // scale given y is the limiting
                                             // factor

      ricky.TargetYVelocity =
          copysign(100.0, (ricky.TargetYAxis - ricky.CurrentYAxis));
      ricky.TargetXVelocity =
          100.0 * ((ricky.TargetXAxis - ricky.CurrentXAxis) /
                   fabs((ricky.TargetYAxis - ricky.CurrentYAxis)));
      ricky.TargetRVelocity =
          50.0 *
          ((TangentialDist) / fabs((ricky.TargetYAxis - ricky.CurrentYAxis)));
    } else { // scale given x is the limiting factor

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
  }
}

int Preliminary_Lock_Detect() { // detect if robot is stuck
  extern Robot_Telemetry ricky;
  double LastX = ricky.CurrentXAxis;
  double LastY = ricky.CurrentYAxis;
  double LastTargetX = ricky.TargetXAxis;
  double LastTargetY = ricky.TargetYAxis;
  double LastTargetTheta = ricky.TargetTheta;
  double LastTheta = ricky.CurrentThetaValue;
  bool LastEngineBusy = false;
  while (true) {
    if ((ricky.EngineBusy && LastEngineBusy) &&
        (LastTargetX == ricky.TargetXAxis && LastTargetY == ricky.TargetYAxis &&
         LastTargetTheta == ricky.TargetTheta)) {
      if (fabs(LastX - ricky.CurrentXAxis) < ricky.StuckTolerance &&
          fabs(LastY - ricky.CurrentYAxis) < ricky.StuckTolerance &&
          fabs(LastTheta - ricky.CurrentThetaValue) < ricky.StuckTolerance &&
          (ricky.TargetRVelocity != 0 || ricky.TargetXVelocity != 0 ||
           ricky.TargetYVelocity != 0)) {
        ricky.TravelImpeded = true;
        // ricky.TargetXAxis = ricky.CurrentXAxis;
        // ricky.TargetYAxis = ricky.CurrentYAxis;
        // ricky.TargetTheta = ricky.CurrentThetaValue;
        printf("Stuck");
      } else {
        ricky.TravelImpeded = false;
      }

    } else if (!ricky.EngineBusy && LastEngineBusy) {
      ricky.TravelImpeded = false; // if engine is not busy and was last time
    } else if (ricky.EngineBusy && !LastEngineBusy) {
      ricky.TravelImpeded = false; // if engine is busy and was not last time
      vex::task::sleep(1000);      // wait 600ms to let the engine get going
    } else {
      ricky.TravelImpeded = false;
    }
    LastX = ricky.CurrentXAxis;
    LastY = ricky.CurrentYAxis;
    LastTheta = ricky.CurrentThetaValue;
    LastTargetX = ricky.TargetXAxis;
    LastTargetY = ricky.TargetYAxis;
    LastTargetTheta = ricky.TargetTheta;
    LastEngineBusy = ricky.EngineBusy;
    vex::task::sleep(300); // check 3 times a second
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

void SupplementProximityRamp() { // ramp down to target speed based on distance
                                 // to target
  extern Robot_Telemetry ricky;
  double RampDown =
      ((fabs(ricky.TargetXAxis - ricky.CurrentXAxis) +
        fabs(ricky.TargetYAxis - ricky.CurrentYAxis)) +
       fabs(FromGyro(ricky.TargetTheta) * -3.14159265359 * 370 / 180.0)) *
      0.15; // find total distance to target
  RampDown = (RampDown * RampDown) / 3000.0 +
             0.1; // i have no idea how this works at this point. no matter
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

void SupplementMotorVectorEngine() { // main calculation loop
  extern Robot_Telemetry ricky;
  // convert to polar coordinates and rotate by difference of starting point and
  // current theta
  EnginePolarTransformation(ricky.TargetXVelocity, ricky.TargetYVelocity,
                            180 - ricky.CurrentThetaValue);

  DriveMotors(ricky.EngineTransformReturnX, ricky.EngineTransformReturnY,
              ricky.TargetRVelocity,
              ricky.TargetTotalVelocity); // send to motor driver
}

void SupplementDirect_Vector_Generator() { // calculate direct vector to target
                                           // coords
  extern Robot_Telemetry ricky;
  if (fabs(ricky.TargetXAxis - ricky.CurrentXAxis) < ricky.DistanceTolerance &&
      fabs(ricky.TargetYAxis - ricky.CurrentYAxis) < ricky.DistanceTolerance &&
      fabs(ricky.TargetTheta - ricky.CurrentThetaValue) <
          ricky.AngleTolerance) { // within tolerance stop robot to prevent
                                  // over tune
    ricky.TargetXVelocity = 0;
    ricky.TargetYVelocity = 0;
    ricky.TargetRVelocity = 0; // set all velocities to 0 to stop robot
    ricky.EngineBusy = false;  // set engine to not busy
  } else {
    ricky.EngineBusy = true; // set engine to busy
    double TangentialDist = FromGyro(ricky.TargetTheta) * -3.14159265359 * 370 /
                            180.0; // find tangential distance of wheels
                                   // based on radius and heading
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
                     ricky.CurrentYAxis))) { // scale given y is the limiting
                                             // factor

      ricky.TargetYVelocity =
          copysign(100.0, (ricky.TargetYAxis - ricky.CurrentYAxis));
      ricky.TargetXVelocity =
          100.0 * ((ricky.TargetXAxis - ricky.CurrentXAxis) /
                   fabs((ricky.TargetYAxis - ricky.CurrentYAxis)));
      ricky.TargetRVelocity =
          50.0 *
          ((TangentialDist) / fabs((ricky.TargetYAxis - ricky.CurrentYAxis)));
    } else { // scale given x is the limiting factor

      ricky.TargetYVelocity =
          100.0 * ((ricky.TargetYAxis - ricky.CurrentYAxis) /
                   fabs((ricky.TargetXAxis - ricky.CurrentXAxis)));
      ricky.TargetXVelocity =
          copysign(100.0, (ricky.TargetXAxis - ricky.CurrentXAxis));
      ricky.TargetRVelocity =
          50.0 *
          ((TangentialDist) / fabs((ricky.TargetXAxis - ricky.CurrentXAxis)));
    }

    SupplementProximityRamp(); // ramp to target velocity
  }
}

int DriverSupplementEngine() { // Main engine loop
  extern Robot_Telemetry ricky;
  ricky.TargetXAxis = -1850;
  ricky.TargetYAxis = 1220;
  ricky.TargetTheta = 242;
  ricky.TargetSpeed = .7;
  while (true) {
    EncoderIntegral(); // Get odometry from encoders

    if (Controller1.ButtonL2.pressing()) {
      ricky.Override_Manual_R = true;
      SupplementDirect_Vector_Generator(); // Calculate motor powers from inputs
                                           // in Robot_Telemetry structure
      SupplementMotorVectorEngine(); // Calculate motor powers from inputs in
      ricky.FlywheelTargetVelocity = 69;
    } else {
      ricky.Override_Manual_R = false;
    }
    vex::task::sleep(5);
  }
};