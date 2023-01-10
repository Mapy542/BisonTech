#include "MotorDriver.cpp"
#include "Odometry.cpp"
#include "vex.h"
#include <cmath>

// Radial Distance calculator. takes two degrees and returns the distance
// between them will wrap around 360 degrees if needed
double RadialDistance(double Theta1, double Theta2) {
  double distance = Theta1 - Theta2;
  if (distance > 180) {
    distance -= 360;
  } else if (distance < -180) {
    distance += 360;
  }

  return distance;
}

// RadialDistance given theta1 is the gyro reading
double FromGyro(double theta1) {
  return RadialDistance(Gyroscope.heading(degrees), theta1);
}

double AbsoluteCumulativeVelocity() {
  extern Robot_Telemetry ricky;
  return fabs(ricky.CurrentXVelocity) + fabs(ricky.CurrentYVelocity) +
         fabs(ricky.CurrentRVelocity);
}

void MotorVectorEngine() { // main calculation loop
  extern Robot_Telemetry ricky;
  if (ricky.TargetXVelocity != 0 || ricky.TargetYVelocity != 0 ||
      ricky.TargetRVelocity != 0) {
    ricky.EngineBusy = true;
    ricky.SetXVelocity +=
        fmod(ricky.TargetXVelocity - ricky.CurrentXVelocity,
             ricky.MaxAcceleration); // fmod is a modulo function for doubles
    ricky.SetYVelocity += fmod(ricky.TargetYVelocity - ricky.CurrentYVelocity,
                               ricky.MaxAcceleration);
    ricky.SetRVelocity += fmod(ricky.TargetRVelocity - ricky.CurrentRVelocity,
                               ricky.MaxAcceleration);

    DriveMotors(ricky.SetXVelocity, ricky.SetYVelocity, ricky.SetRVelocity, 1);
  } else {
    DriveMotors(0, 0, 0, 0);
    ricky.EngineBusy = false;
  }
}

int Engine() { // Main engine loop
  while (true) {

    EncoderIntegral();   // Get odometry from encoders
    MotorVectorEngine(); // Calculate motor powers from inputs in
                         // Robot_Telemetry structure
    vex::task::sleep(25);
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