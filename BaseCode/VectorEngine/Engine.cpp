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
double AbsoluteCumulativeVelocity() {
  extern Robot_Telemetry ricky;
  return fabs(ricky.CurrentXVelocity) + fabs(ricky.CurrentYVelocity) +
         fabs(ricky.CurrentRVelocity);
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

  // current X and y represent global heading based on field.
  // now convert back to local heading based on robot rotation (inverse of
  // odometry)
  ricky.DriveXPower =
      (ricky.SetYVelocity * sin(Gyroscope.heading(degrees) * M_PI / 180) +
       ricky.SetXVelocity * cos(Gyroscope.heading(degrees) * M_PI / 180)) *
      -1; // why must this be flipped? dunno
  ricky.DriveYPower =
      (ricky.SetYVelocity * cos(Gyroscope.heading(degrees) * M_PI / 180) +
       ricky.SetXVelocity * sin(Gyroscope.heading(degrees) * M_PI / 180)) *
      -1;

  DriveMotors(ricky.DriveXPower, ricky.DriveYPower, ricky.SetRVelocity,
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
        FromGyro(ricky.TargetTheta) * -3.14159265359 * 370 / 180.0 /
        3; // find tangential distance of wheels based on radius and heading
    if (fabs((ricky.TargetXAxis - ricky.CurrentXAxis)) <
            fabs((TangentialDist)) &&
        fabs((ricky.TargetYAxis - ricky.CurrentYAxis)) <
            fabs((TangentialDist))) { // if tangential distance is more than x
                                      // and y distance

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
    double RampDown = ((fabs((ricky.TargetXAxis - ricky.CurrentXAxis)) +
                        fabs((ricky.TargetYAxis - ricky.CurrentYAxis))) +
                       fabs((TangentialDist))) *
                      0.15;
    // printf("%.6f", RampDown);
    // printf("\n");
    RampDown = (RampDown * RampDown) /
               500.0; // 15 defines the floor or minimum value for
                      // ramp down. At min, 15% of max speed.
    if (1.0 < RampDown) {
      RampDown = 1.0;
    }
    ricky.TargetTotalVelocity = ricky.TargetSpeed * RampDown;
  }
}

int Engine() { // Main engine loop
  while (true) {
    EncoderIntegral();         // Get odometry from encoders
    Direct_Vector_Generator(); // Calculate motor powers from inputs in
                               // Robot_Telemetry structure
    // MotorVectorEngine();       // Calculate motor powers from inputs in
    vex::task::sleep(5);
  }
};

/*
//crappycrappy crap
 extern Robot_Telemetry ricky;

  if ((fabs(ricky.TargetXAxis - ricky.ricky.CurrentXAxis) >
       ricky.DistanceTolerance) ||
      (fabs(ricky.TargetYAxis - ricky.ricky.CurrentYAxis) >
       ricky.DistanceTolerance) ||
      (fabs(ricky.TargetTheta - ricky.CurrentThetaValue) >
       ricky.AngleTolerance)) { // if the robot is out of target range +
                                // tolerance then run calculations

    double DriveXScale;
    double DriveYScale;
    double DriveRScale;

    if (!ricky.Engine_Busy) {
      ricky.Engine_Busy =
          true; // Set busy flag to true so autonomous code can wait
      ricky.Busy_Start_Time = (double)vex::timer::system()

                                  ricky.StartingXAxis = ricky.CurrentXAxis;
      ricky.StartingYAxis = ricky.CurrentYAxis;
      ricky.StartingTheta = ricky.CurrentThetaValue;
    }

    TangentialDist = 3.0 * FromGyro(ricky.TargetTheta) *
                     6.2831; // find tangential distance to linearize rotation
                             // to the same scale as ricky.TargetX and
                             // ricky.TargetY. Then increase the scale by 3 to
                             // make it more sensitive to rotation.

    if (ricky.TravelStyle == 0) { // Calculate Scale for direct line
      if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < fabs((TangentialDist)) &&
          fabs((ricky.TargetY - ricky.CurrentYAxis)) <
              fabs((TangentialDist))) { // if tangential distance is more than x
                                        // and y distance

        DriveYScale =
            100.0 * ((ricky.TargetY - ricky.CurrentYAxis) /
                     fabs((TangentialDist))); // scale y to y over tangential
        DriveXScale =
            100.0 * ((ricky.TargetX - ricky.CurrentXAxis) /
                     fabs((TangentialDist))); // scale x to x over tangential
        DriveRScale =
            50.0; // scale r to 50% as r is 200% more powerful than x and y.

      } else if (fabs((ricky.TargetX - ricky.CurrentXAxis)) <
                 fabs((ricky.TargetY -
                       ricky.CurrentYAxis))) { // scale given x is the limiting
                                               // factor

        DriveYScale = 100.0;
        DriveXScale = 100.0 * ((ricky.TargetX - ricky.CurrentXAxis) /
                               fabs((ricky.TargetY - ricky.CurrentYAxis)));
        DriveRScale = 50.0 * ((TangentialDist) /
                              fabs((ricky.TargetY - ricky.CurrentYAxis)));
      } else { // scale given y is the limiting factor

        DriveYScale = 100.0 * ((ricky.TargetY - ricky.CurrentYAxis) /
                               fabs((ricky.TargetX - ricky.CurrentXAxis)));
        DriveXScale = 100.0;
        DriveRScale = 50.0 * ((TangentialDist) /
                              fabs((ricky.TargetX - ricky.CurrentXAxis)));
      }
    } else if (ricky.TravelStyle == 1) { // calculate scale for x first arc

    } else if (ricky.TravelStyle == 2) { // calculate scale for y first arc
    }

    // Calculate motor powers over trig values
    double DriveXPower =
        DriveYScale * sin(Gyroscope.heading(degrees) * M_PI / 180) +
        DriveXScale * cos(Gyroscope.heading(degrees) * M_PI / 180);
    double DriveYPower =
        DriveYScale * cos(Gyroscope.heading(degrees) * M_PI / 180) +
        DriveXScale * sin(Gyroscope.heading(degrees) * M_PI / 180);

    // Apply motor powers to individual motors
    DriveMotors(DriveXPower, DriveYPower, DriveRPower, ricky.TargetSpeed);

    if ((double)vex::timer::system() - ricky.BusyStartTime > 250 &&
        AbsoluteCumulativeVelocity() < ricky.TargetSpeed * 0.1) {
      ricky.Travel_Impeded = true;
    }

    if (breakcount > 100 && RampUp > .7 && HardCurve > 0.7) {
      DriveMotors(0, 0, 0, 0);
      break;
    }
  } else {
    if (!ricky.Engine_Busy) {
      ricky.Engine_Busy =
          false; // Set busy flag to false so autonomous code can
      // continue
      ricky.Travel_Impeded = false;
    }
    DriveMotors(0, 0, 0, 0);
  }
// Precision for stopping at a place
void Destination(double ricky.TargetX, double ricky.TargetY, double r, double
speed)
{
  extern double ricky.CurrentXAxis;
  extern double ricky.CurrentYAxis;
  extern double globaldelta;

  float DriveYScale = 0;
  float DriveXScale = 0;
  float DriveRScale = 0;
  float DriveYDir = 0;
  float DriveXDir = 0;
  float DriveYPower = 0;
  float DriveXPower = 0;
  float DriveRPower = 0;

  float RampUp = 0.01;
  float HardCurve = 0.01;
  float RampDown = 1.0;

  int breakcount = 0;
  double TangentialDist = 3.0 * (FromGyro(r) * 6.2831);

  if (0.15 < FL.velocity(percent) || (0.15 < RL.velocity(percent) || (0.15 <
FR.velocity(percent) || 0.15 < RR.velocity(percent))))
  {
    RampUp = 1.0;
  }

  while (!((fabs((ricky.TargetY - ricky.CurrentYAxis)) < 10.0 &&
fabs((TangentialDist)) < 10.0)
&& fabs((ricky.TargetX - ricky.CurrentXAxis)) < 10.0))
  {

    // Calculate Odometry
    Poll_Absolute_Cords();
    TangentialDist = 3.0 * (FromGyro(r) * 6.2831);

    // Calculate Ramp Powers
    RampUp = RampUp * 1.06;
    if (1.0 < RampUp)
    {
      RampUp = 1.0;
    }

    // Gives Time to ramp up speed before collision check regardless of
previous speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((ricky.TargetX - ricky.CurrentXAxis)) +
fabs((ricky.TargetY - ricky.CurrentYAxis))) + 1.5 * fabs((TangentialDist))) *
-0.015; RampDown = (RampDown * RampDown + 15.0) / 100.0; // 15 defines the floor
or minimum value for ramp down. At min, 15% of max speed. if (1.0 < RampDown)
    {
      RampDown = 1.0;
    }

    // Calculate Scale for direct line
    if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < fabs((TangentialDist)) &&
fabs((ricky.TargetY - ricky.CurrentYAxis)) < fabs((TangentialDist)))
    {
      DriveYScale = 100.0 * (fabs((ricky.TargetY - ricky.CurrentYAxis)) /
fabs((TangentialDist))); DriveXScale = 100.0 * (fabs((ricky.TargetX -
ricky.CurrentXAxis)) / fabs((TangentialDist))); DriveRScale = 50.0;
    }
    else
    {
      if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < fabs((ricky.TargetY -
ricky.CurrentYAxis)))
      {
        DriveYScale = 100.0;
        DriveXScale = 100.0 * (fabs((ricky.TargetX - ricky.CurrentXAxis)) /
fabs((ricky.TargetY - ricky.CurrentYAxis))); DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((ricky.TargetY - ricky.CurrentYAxis)));
      }
      else
      {
        DriveYScale = 100.0 * (fabs((ricky.TargetY - ricky.CurrentYAxis)) /
fabs((ricky.TargetX - ricky.CurrentXAxis))); DriveXScale = 100.0; DriveRScale
= 50.0 * (fabs((TangentialDist)) / fabs((ricky.TargetX - ricky.CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < 10.0)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (ricky.TargetX - ricky.CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((ricky.TargetY - ricky.CurrentYAxis)) < 10.0)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (ricky.TargetY - ricky.CurrentYAxis < 0.0)
      {
        DriveYDir = 1.0 * DriveYScale;
      }
      else
      {
        DriveYDir = -1.0 * DriveYScale;
      }
    }
    if (fabs((TangentialDist)) < 10.0)
    {
      DriveRPower = 0.0;
    }
    else
    {
      if (TangentialDist < 0.0)
      {
        DriveRPower = 1.0 * DriveRScale;
      }
      else
      {
        DriveRPower = -1.0 * DriveRScale;
      }
    }

    // Calculate motor powers over trig values
    DriveXPower = (DriveYDir * sin(Gyroscope.heading(degrees) * M_PI / 180) +
(DriveXDir * cos(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0) * 1.0;
    DriveYPower = (DriveYDir * cos(Gyroscope.heading(degrees) * M_PI / 180) +
DriveXDir * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 1.0;
    // Apply motor powers to individual motors
    DriveMotors(DriveXPower, DriveYPower, DriveRPower, speed * (RampUp *
RampDown));
    // Print_XYR();
    if (globaldelta < (0.001 * speed * RampDown))
    {
      breakcount++;
    }
    else
    {
      breakcount = 0;
    }

    if (breakcount > 100 && RampUp > .7 && HardCurve > 0.7)
    {
      DriveMotors(0, 0, 0, 0);
      break;
    }
    wait(5, msec);
  }
  DriveMotors(0.0, 0.0, 0.0, 0.0);
}

// Speedy for navigating around obstacles
void Waypoint(double ricky.TargetX, double ricky.TargetY, double r, double
speed)
{
  extern double ricky.CurrentXAxis;
  extern double ricky.CurrentYAxis;
  extern double globaldelta;

  float DriveYScale = 0;
  float DriveXScale = 0;
  float DriveRScale = 0;
  float DriveYDir = 0;
  float DriveXDir = 0;
  float DriveYPower = 0;
  float DriveXPower = 0;
  float DriveRPower = 0;

  float RampUp = 0.01;
  float HardCurve = 0.01;
  float RampDown = 1;

  int breakcount = 0;

  double TangentialDist = 3.0 * (FromGyro(r) * 6.2831);
  if (0.15 < FL.velocity(percent) || (0.15 < RL.velocity(percent) || (0.15 <
FR.velocity(percent) || 0.15 < RR.velocity(percent))))
  {
    RampUp = 1.0;
  }

  while (!((fabs((ricky.TargetY - ricky.CurrentYAxis)) < 50.0 &&
fabs((TangentialDist)) < 50.0)
&& fabs((ricky.TargetX - ricky.CurrentXAxis)) < 50.0))
  {

    // Calculate Odometry
    Poll_Absolute_Cords();
    TangentialDist = 3.0 * (FromGyro(r) * 6.2831);

    // Calculate Ramp Powers
    RampUp = RampUp * 1.06;
    if (1.0 < RampUp)
    {
      RampUp = 1.0;
    }

    // Gives Time to ramp up speed before collision check regardless of
previous speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((ricky.TargetX - ricky.CurrentXAxis)) +
fabs((ricky.TargetY - ricky.CurrentYAxis))) + 1.5 * fabs((TangentialDist))) *
-0.015; RampDown = (RampDown * RampDown + 50.0) / 100.0; // Ramp down to at max
50% if (1.0 < RampDown)
    {
      RampDown = 1.0;
    }

    // Calculate Scale for direct line
    if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < fabs((TangentialDist)) &&
fabs((ricky.TargetY - ricky.CurrentYAxis)) < fabs((TangentialDist)))
    {
      DriveYScale = 100.0 * (fabs((ricky.TargetY - ricky.CurrentYAxis)) /
fabs((TangentialDist))); DriveXScale = 100.0 * (fabs((ricky.TargetX -
ricky.CurrentXAxis)) / fabs((TangentialDist))); DriveRScale = 50.0;
    }
    else
    {
      if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < fabs((ricky.TargetY -
ricky.CurrentYAxis)))
      {
        DriveYScale = 100.0;
        DriveXScale = 100.0 * (fabs((ricky.TargetX - ricky.CurrentXAxis)) /
fabs((ricky.TargetY - ricky.CurrentYAxis))); DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((ricky.TargetY - ricky.CurrentYAxis)));
      }
      else
      {
        DriveYScale = 100.0 * (fabs((ricky.TargetY - ricky.CurrentYAxis)) /
fabs((ricky.TargetX - ricky.CurrentXAxis))); DriveXScale = 100.0; DriveRScale
= 50.0 * (fabs((TangentialDist)) / fabs((ricky.TargetX - ricky.CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < 50.0)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (ricky.TargetX - ricky.CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((ricky.TargetY - ricky.CurrentYAxis)) < 50.0)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (ricky.TargetY - ricky.CurrentYAxis < 0.0)
      {
        DriveYDir = 1.0 * DriveYScale;
      }
      else
      {
        DriveYDir = -1.0 * DriveYScale;
      }
    }
    if (fabs((TangentialDist)) < 50.0)
    {
      DriveRPower = 0.0;
    }
    else
    {
      if (TangentialDist < 0.0)
      {
        DriveRPower = 1.0 * DriveRScale;
      }
      else
      {
        DriveRPower = -1.0 * DriveRScale;
      }
    }

    // Calculate motor powers over trig values
    DriveXPower = (DriveYDir * sin(Gyroscope.heading(degrees) * M_PI / 180) +
(DriveXDir * cos(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0) * 1.0;
    DriveYPower = (DriveYDir * cos(Gyroscope.heading(degrees) * M_PI / 180) +
DriveXDir * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 1.0;
    // Apply motor powers to individual motors
    DriveMotors(DriveXPower, DriveYPower, DriveRPower, speed * (RampUp *
RampDown));
    // Print_XYR();

    if (globaldelta < (0.001 * speed * RampDown))
    {
      breakcount++;
    }
    else
    {
      breakcount = 0;
    }

    if (breakcount > 100 && RampUp > .5 && HardCurve > .5)
    {
      break;
    }
    wait(5, msec);
  }
  DriveMotors(0.0, 0.0, 0.0, 0.0);
}

// drivetype true: destination false: waypoint
// Draws linear arc that starts in the ricky.TargetX direction
void XArc(double ricky.TargetX, double ricky.TargetY, double r, double speed,
bool drivetype)
{
  extern double ricky.CurrentXAxis;
  extern double ricky.CurrentYAxis;
  extern double globaldelta;

  double initXLoc = ricky.CurrentXAxis;
  float xCompleationRatio = 0;

  float DriveYScale = 0;
  float DriveXScale = 0;
  float DriveRScale = 0;
  float DriveYDir = 0;
  float DriveXDir = 0;
  float DriveYPower = 0;
  float DriveXPower = 0;
  float DriveRPower = 0;

  float RampUp = 0.01;
  float HardCurve = 0.01;
  float RampDown = 1.0;

  float increasediffs;

  if (drivetype)
  {
    increasediffs = 0;
  }
  else
  {
    increasediffs = 40;
  }

  int breakcount = 0;
  double TangentialDist = 2.0 * (FromGyro(r) * 6.2831);

  if (0.15 < FL.velocity(percent) || (0.15 < RL.velocity(percent) || (0.15 <
FR.velocity(percent) || 0.15 < RR.velocity(percent))))
  {
    RampUp = 1.0;
  }

  while (!(fabs((ricky.TargetY - ricky.CurrentYAxis)) < (10.0 + increasediffs)
&& fabs((TangentialDist)) < (10.0 + increasediffs) && fabs((ricky.TargetX -
ricky.CurrentXAxis)) < 10.0 + (increasediffs)))
  {

    // Calculate Odometry
    Poll_Absolute_Cords();
    TangentialDist = 2.0 * (FromGyro(r) * 6.2831);

    // Calculate Ramp Powers
    RampUp = RampUp * 1.06;
    if (1.0 < RampUp)
    {
      RampUp = 1.0;
    }

    // Gives Time to ramp up speed before collision check regardless of
previous speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((ricky.TargetX - ricky.CurrentXAxis)) +
fabs((ricky.TargetY - ricky.CurrentYAxis))) + 1.5 * fabs((TangentialDist))) *
-0.015; RampDown = (RampDown * RampDown + 15.0) / 100.0; // 15 defines the floor
or minimum value for ramp down. At min, 15% of max speed. if (1.0 < RampDown ||
drivetype == false)
    {
      RampDown = 1.0;
    }

    // figure out how close is ricky.TargetX to on line
    xCompleationRatio = (1 - fabs(ricky.TargetX - ricky.CurrentXAxis) /
fabs(ricky.TargetX - initXLoc)) * 100;

    // Calculate Scale for hitting ricky.TargetX dest., then hitting
ricky.TargetY dest. if (fabs((ricky.TargetX - ricky.CurrentXAxis)) <
fabs((TangentialDist)) && fabs((ricky.TargetY - ricky.CurrentYAxis)) <
fabs((TangentialDist)))
    {
      DriveYScale = xCompleationRatio * (fabs((ricky.TargetY -
ricky.CurrentYAxis)) / fabs((TangentialDist))); DriveXScale = 100.0 *
(fabs((ricky.TargetX - ricky.CurrentXAxis)) / fabs((TangentialDist)));
DriveRScale = 50.0;
    }
    else
    {
      if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < fabs((ricky.TargetY -
ricky.CurrentYAxis)))
      {
        DriveYScale = xCompleationRatio;
        DriveXScale = 100.0 * (fabs((ricky.TargetX - ricky.CurrentXAxis)) /
fabs((ricky.TargetY - ricky.CurrentYAxis))); DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((ricky.TargetY - ricky.CurrentYAxis)));
      }
      else
      {
        DriveYScale = xCompleationRatio * (fabs((ricky.TargetY -
ricky.CurrentYAxis)) / fabs((ricky.TargetX
- ricky.CurrentXAxis))); DriveXScale = 100.0; DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((ricky.TargetX - ricky.CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < 10.0 + increasediffs)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (ricky.TargetX - ricky.CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((ricky.TargetY - ricky.CurrentYAxis)) < 10.0 + increasediffs)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (ricky.TargetY - ricky.CurrentYAxis < 0.0)
      {
        DriveYDir = 1.0 * DriveYScale;
      }
      else
      {
        DriveYDir = -1.0 * DriveYScale;
      }
    }
    if (fabs((TangentialDist)) < 10.0 + increasediffs)
    {
      DriveRPower = 0.0;
    }
    else
    {
      if (TangentialDist < 0.0)
      {
        DriveRPower = 1.0 * DriveRScale;
      }
      else
      {
        DriveRPower = -1.0 * DriveRScale;
      }
    }

    // Calculate motor powers over trig values
    DriveXPower = (DriveYDir * sin(Gyroscope.heading(degrees) * M_PI / 180) +
(DriveXDir * cos(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0) * 1.0;
    DriveYPower = (DriveYDir * cos(Gyroscope.heading(degrees) * M_PI / 180) +
DriveXDir * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 1.0;
    // Apply motor powers to individual motors
    DriveMotors(DriveXPower, DriveYPower, DriveRPower, speed * (RampUp *
RampDown));
    // Print_XYR();
    if (globaldelta < (0.001 * speed * RampDown))
    {
      breakcount++;
    }
    else
    {
      breakcount = 0;
    }

    if (breakcount > 100 && RampUp > .7 && HardCurve > 0.7)
    {
      DriveMotors(0, 0, 0, 0);
      break;
    }
    wait(5, msec);
  }
  if (drivetype)
  {
    DriveMotors(0.0, 0.0, 0.0, 0.0);
  }
}

// drivetype true: destination false: waypoint
// Draws a linear arc that starts in the ricky.TargetY direction
void YArc(double ricky.TargetX, double ricky.TargetY, double r, double speed,
bool drivetype)
{
  extern double ricky.CurrentXAxis;
  extern double ricky.CurrentYAxis;
  extern double globaldelta;

  double initYLoc = ricky.CurrentYAxis;
  float YCompleationRatio = 0;

  float DriveYScale = 0;
  float DriveXScale = 0;
  float DriveRScale = 0;
  float DriveYDir = 0;
  float DriveXDir = 0;
  float DriveYPower = 0;
  float DriveXPower = 0;
  float DriveRPower = 0;

  float RampUp = 0.01;
  float HardCurve = 0.01;
  float RampDown = 1.0;

  float increasediffs;

  if (drivetype)
  {
    increasediffs = 0;
  }
  else
  {
    increasediffs = 40;
  }

  int breakcount = 0;
  double TangentialDist = 2.0 * (FromGyro(r) * 6.2831);

  if (0.15 < FL.velocity(percent) || (0.15 < RL.velocity(percent) || (0.15 <
FR.velocity(percent) || 0.15 < RR.velocity(percent))))
  {
    RampUp = 1.0;
  }

  while (!(fabs((ricky.TargetY - ricky.CurrentYAxis)) < (10.0 + increasediffs)
&& fabs((TangentialDist)) < (10.0 + increasediffs) && fabs((ricky.TargetX -
ricky.CurrentXAxis)) < 10.0 + (increasediffs)))
  {

    // Calculate Odometry
    Poll_Absolute_Cords();
    TangentialDist = 2.0 * (FromGyro(r) * 6.2831);

    // Calculate Ramp Powers
    RampUp = RampUp * 1.06;
    if (1.0 < RampUp)
    {
      RampUp = 1.0;
    }

    // Gives Time to ramp up speed before collision check regardless of
previous speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((ricky.TargetX - ricky.CurrentXAxis)) +
fabs((ricky.TargetY - ricky.CurrentYAxis))) + 1.5 * fabs((TangentialDist))) *
-0.015; RampDown = (RampDown * RampDown + 15.0) / 100.0; // 15 defines the floor
or minimum value for ramp down. At min, 15% of max speed. if (1.0 < RampDown ||
drivetype == false)
    {
      RampDown = 1.0;
    }

    // figure out how close is ricky.TargetX to on line
    YCompleationRatio = (1 - fabs(ricky.TargetY - ricky.CurrentYAxis) /
fabs(ricky.TargetY - initYLoc)) * 100;

    // Calculate Scale for hitting ricky.TargetX dest., then hitting
ricky.TargetY dest. if (fabs((ricky.TargetX - ricky.CurrentXAxis)) <
fabs((TangentialDist)) && fabs((ricky.TargetY - ricky.CurrentYAxis)) <
fabs((TangentialDist)))
    {
      DriveYScale = 100.0 * (fabs((ricky.TargetY - ricky.CurrentYAxis)) /
fabs((TangentialDist))); DriveXScale = YCompleationRatio * (fabs((ricky.TargetX
- ricky.CurrentXAxis)) / fabs((TangentialDist))); DriveRScale = 50.0;
    }
    else
    {
      if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < fabs((ricky.TargetY -
ricky.CurrentYAxis)))
      {
        DriveYScale = 100.0;
        DriveXScale = YCompleationRatio * (fabs((ricky.TargetX -
ricky.CurrentXAxis)) / fabs((ricky.TargetY
- ricky.CurrentYAxis))); DriveRScale = 50.0 * (fabs((TangentialDist)) /
fabs((ricky.TargetY - ricky.CurrentYAxis)));
      }
      else
      {
        DriveYScale = 100.0 * (fabs((ricky.TargetY - ricky.CurrentYAxis)) /
fabs((ricky.TargetX - ricky.CurrentXAxis))); DriveXScale = YCompleationRatio;
DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((ricky.TargetX -
ricky.CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < 10.0 + increasediffs)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (ricky.TargetX - ricky.CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((ricky.TargetY - ricky.CurrentYAxis)) < 10.0 + increasediffs)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (ricky.TargetY - ricky.CurrentYAxis < 0.0)
      {
        DriveYDir = 1.0 * DriveYScale;
      }
      else
      {
        DriveYDir = -1.0 * DriveYScale;
      }
    }
    if (fabs((TangentialDist)) < 10.0 + increasediffs)
    {
      DriveRPower = 0.0;
    }
    else
    {
      if (TangentialDist < 0.0)
      {
        DriveRPower = 1.0 * DriveRScale;
      }
      else
      {
        DriveRPower = -1.0 * DriveRScale;
      }
    }

    // Calculate motor powers over trig values
    DriveXPower = (DriveYDir * sin(Gyroscope.heading(degrees) * M_PI / 180) +
(DriveXDir * cos(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0) * 1.0;
    DriveYPower = (DriveYDir * cos(Gyroscope.heading(degrees) * M_PI / 180) +
DriveXDir * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 1.0;
    // Apply motor powers to individual motors
    DriveMotors(DriveXPower, DriveYPower, DriveRPower, speed * (RampUp *
RampDown));
    // Print_XYR();
    if (globaldelta < (0.001 * speed * RampDown))
    {
      breakcount++;
    }
    else
    {
      breakcount = 0;
    }

    if (breakcount > 100 && RampUp > .7 && HardCurve > 0.7)
    {
      DriveMotors(0, 0, 0, 0);
      break;
    }
    wait(5, msec);
  }
  if (drivetype)
  {
    DriveMotors(0.0, 0.0, 0.0, 0.0);
  }
}

void Ram(double ricky.TargetX, double ricky.TargetY, double r)
{
  extern double ricky.CurrentXAxis;
  extern double ricky.CurrentYAxis;
  extern double globaldelta;

  float DriveYScale = 0;
  float DriveXScale = 0;
  float DriveRScale = 0;
  float DriveYDir = 0;
  float DriveXDir = 0;
  float DriveYPower = 0;
  float DriveXPower = 0;
  float DriveRPower = 0;

  float RampUp = 0.01;
  float HardCurve = 0.01;
  float RampDown = 1;

  int breakcount = 0;

  double TangentialDist = 2.0 * (FromGyro(r) * 6.2831);
  if (0.15 < FL.velocity(percent) || (0.15 < RL.velocity(percent) || (0.15 <
FR.velocity(percent) || 0.15 < RR.velocity(percent))))
  {
    RampUp = 1.0;
  }

  while (!((fabs((ricky.TargetY - ricky.CurrentYAxis)) < 50.0 &&
fabs((TangentialDist)) < 50.0)
&& fabs((ricky.TargetX - ricky.CurrentXAxis)) < 50.0))
  {

    // Calculate Odometry
    Poll_Absolute_Cords();
    TangentialDist = 2.0 * (FromGyro(r) * 6.2831);

    // Calculate Ramp Powers
    RampUp = RampUp * 1.06;
    if (1.0 < RampUp)
    {
      RampUp = 1.0;
    }

    // Gives Time to ramp up speed before collision check regardless of
previous speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((ricky.TargetX - ricky.CurrentXAxis)) +
fabs((ricky.TargetY - ricky.CurrentYAxis))) + 1.5 * fabs((TangentialDist))) *
-0.015; RampDown = (RampDown * RampDown + 50.0) / 100.0; // Ramp down to at max
50% if (1.0 < RampDown)
    {
      RampDown = 1.0;
    }

    // Calculate Scale for direct line
    if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < fabs((TangentialDist)) &&
fabs((ricky.TargetY - ricky.CurrentYAxis)) < fabs((TangentialDist)))
    {
      DriveYScale = 100.0 * (fabs((ricky.TargetY - ricky.CurrentYAxis)) /
fabs((TangentialDist))); DriveXScale = 100.0 * (fabs((ricky.TargetX -
ricky.CurrentXAxis)) / fabs((TangentialDist))); DriveRScale = 50.0;
    }
    else
    {
      if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < fabs((ricky.TargetY -
ricky.CurrentYAxis)))
      {
        DriveYScale = 100.0;
        DriveXScale = 100.0 * (fabs((ricky.TargetX - ricky.CurrentXAxis)) /
fabs((ricky.TargetY - ricky.CurrentYAxis))); DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((ricky.TargetY - ricky.CurrentYAxis)));
      }
      else
      {
        DriveYScale = 100.0 * (fabs((ricky.TargetY - ricky.CurrentYAxis)) /
fabs((ricky.TargetX - ricky.CurrentXAxis))); DriveXScale = 100.0; DriveRScale
= 50.0 * (fabs((TangentialDist)) / fabs((ricky.TargetX - ricky.CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((ricky.TargetX - ricky.CurrentXAxis)) < 50.0)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (ricky.TargetX - ricky.CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((ricky.TargetY - ricky.CurrentYAxis)) < 50.0)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (ricky.TargetY - ricky.CurrentYAxis < 0.0)
      {
        DriveYDir = 1.0 * DriveYScale;
      }
      else
      {
        DriveYDir = -1.0 * DriveYScale;
      }
    }
    if (fabs((TangentialDist)) < 50.0)
    {
      DriveRPower = 0.0;
    }
    else
    {
      if (TangentialDist < 0.0)
      {
        DriveRPower = 1.0 * DriveRScale;
      }
      else
      {
        DriveRPower = -1.0 * DriveRScale;
      }
    }

    // Calculate motor powers over trig values
    DriveXPower = (DriveYDir * sin(Gyroscope.heading(degrees) * M_PI / 180) +
(DriveXDir * cos(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0) * 1.0;
    DriveYPower = (DriveYDir * cos(Gyroscope.heading(degrees) * M_PI / 180) +
DriveXDir * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 1.0;
    // Apply motor powers to individual motors
    DriveMotors(DriveXPower, DriveYPower, DriveRPower, .7);
    // Print_XYR();

    if (globaldelta < (0.001 * .7 * RampDown))
    {
      breakcount++;
    }
    else
    {
      breakcount = 0;
    }

    if (breakcount > 100 && RampUp > .5 && HardCurve > .5)
    {
      break;
    }
    wait(5, msec);
  }
  DriveMotors(0.0, 0.0, 0.0, 0.0);
}*/