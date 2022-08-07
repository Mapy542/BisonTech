#include "..\AuxiliaryFunctions\Telemetry.cpp"
#include "MotorDriver.cpp"
#include "Odometry.cpp"
#include "vex.h"

void Engine() {
  // extern struct robot; // import-ish the global robot data

  // while (fabs(CurrentXAxis - x) < 10 && fabs(CurrentYAxis - y) < 10 &&
  // fabs(Gyroscope.heading(degrees) - r) < 3 && fabs(Current))
  //{

  vex::task::sleep(25); // allow other tasks to operate updates 40hz ideally
  //}
};

/*
// Precision for stopping at a place
void Destination(double x, double y, double r, double speed)
{
  extern double CurrentXAxis;
  extern double CurrentYAxis;
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

  while (!((fabs((y - CurrentYAxis)) < 10.0 && fabs((TangentialDist)) < 10.0) &&
fabs((x - CurrentXAxis)) < 10.0))
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

    // Gives Time to ramp up speed before collision check regardless of previous
speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((x - CurrentXAxis)) + fabs((y - CurrentYAxis))) + 1.5 *
fabs((TangentialDist))) * -0.015; RampDown = (RampDown * RampDown + 15.0) /
100.0; // 15 defines the floor or minimum value for ramp down. At min, 15% of
max speed. if (1.0 < RampDown)
    {
      RampDown = 1.0;
    }

    // Calculate Scale for direct line
    if (fabs((x - CurrentXAxis)) < fabs((TangentialDist)) && fabs((y -
CurrentYAxis)) < fabs((TangentialDist)))
    {
      DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((TangentialDist)));
      DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((TangentialDist)));
      DriveRScale = 50.0;
    }
    else
    {
      if (fabs((x - CurrentXAxis)) < fabs((y - CurrentYAxis)))
      {
        DriveYScale = 100.0;
        DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((y -
CurrentYAxis))); DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((y -
CurrentYAxis)));
      }
      else
      {
        DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((x -
CurrentXAxis))); DriveXScale = 100.0; DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((x - CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((x - CurrentXAxis)) < 10.0)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (x - CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((y - CurrentYAxis)) < 10.0)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (y - CurrentYAxis < 0.0)
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
void Waypoint(double x, double y, double r, double speed)
{
  extern double CurrentXAxis;
  extern double CurrentYAxis;
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

  while (!((fabs((y - CurrentYAxis)) < 50.0 && fabs((TangentialDist)) < 50.0) &&
fabs((x - CurrentXAxis)) < 50.0))
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

    // Gives Time to ramp up speed before collision check regardless of previous
speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((x - CurrentXAxis)) + fabs((y - CurrentYAxis))) + 1.5 *
fabs((TangentialDist))) * -0.015; RampDown = (RampDown * RampDown + 50.0) /
100.0; // Ramp down to at max 50% if (1.0 < RampDown)
    {
      RampDown = 1.0;
    }

    // Calculate Scale for direct line
    if (fabs((x - CurrentXAxis)) < fabs((TangentialDist)) && fabs((y -
CurrentYAxis)) < fabs((TangentialDist)))
    {
      DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((TangentialDist)));
      DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((TangentialDist)));
      DriveRScale = 50.0;
    }
    else
    {
      if (fabs((x - CurrentXAxis)) < fabs((y - CurrentYAxis)))
      {
        DriveYScale = 100.0;
        DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((y -
CurrentYAxis))); DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((y -
CurrentYAxis)));
      }
      else
      {
        DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((x -
CurrentXAxis))); DriveXScale = 100.0; DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((x - CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((x - CurrentXAxis)) < 50.0)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (x - CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((y - CurrentYAxis)) < 50.0)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (y - CurrentYAxis < 0.0)
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
// Draws linear arc that starts in the x direction
void XArc(double x, double y, double r, double speed, bool drivetype)
{
  extern double CurrentXAxis;
  extern double CurrentYAxis;
  extern double globaldelta;

  double initXLoc = CurrentXAxis;
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

  while (!(fabs((y - CurrentYAxis)) < (10.0 + increasediffs) &&
fabs((TangentialDist)) < (10.0 + increasediffs) && fabs((x - CurrentXAxis))
< 10.0 + (increasediffs)))
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

    // Gives Time to ramp up speed before collision check regardless of previous
speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((x - CurrentXAxis)) + fabs((y - CurrentYAxis))) + 1.5 *
fabs((TangentialDist))) * -0.015; RampDown = (RampDown * RampDown + 15.0) /
100.0; // 15 defines the floor or minimum value for ramp down. At min, 15% of
max speed. if (1.0 < RampDown || drivetype == false)
    {
      RampDown = 1.0;
    }

    // figure out how close is x to on line
    xCompleationRatio = (1 - fabs(x - CurrentXAxis) / fabs(x - initXLoc)) * 100;

    // Calculate Scale for hitting x dest., then hitting y dest.
    if (fabs((x - CurrentXAxis)) < fabs((TangentialDist)) && fabs((y -
CurrentYAxis)) < fabs((TangentialDist)))
    {
      DriveYScale = xCompleationRatio * (fabs((y - CurrentYAxis)) /
fabs((TangentialDist))); DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) /
fabs((TangentialDist))); DriveRScale = 50.0;
    }
    else
    {
      if (fabs((x - CurrentXAxis)) < fabs((y - CurrentYAxis)))
      {
        DriveYScale = xCompleationRatio;
        DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((y -
CurrentYAxis))); DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((y -
CurrentYAxis)));
      }
      else
      {
        DriveYScale = xCompleationRatio * (fabs((y - CurrentYAxis)) / fabs((x -
CurrentXAxis))); DriveXScale = 100.0; DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((x - CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((x - CurrentXAxis)) < 10.0 + increasediffs)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (x - CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((y - CurrentYAxis)) < 10.0 + increasediffs)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (y - CurrentYAxis < 0.0)
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
// Draws a linear arc that starts in the y direction
void YArc(double x, double y, double r, double speed, bool drivetype)
{
  extern double CurrentXAxis;
  extern double CurrentYAxis;
  extern double globaldelta;

  double initYLoc = CurrentYAxis;
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

  while (!(fabs((y - CurrentYAxis)) < (10.0 + increasediffs) &&
fabs((TangentialDist)) < (10.0 + increasediffs) && fabs((x - CurrentXAxis))
< 10.0 + (increasediffs)))
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

    // Gives Time to ramp up speed before collision check regardless of previous
speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((x - CurrentXAxis)) + fabs((y - CurrentYAxis))) + 1.5 *
fabs((TangentialDist))) * -0.015; RampDown = (RampDown * RampDown + 15.0) /
100.0; // 15 defines the floor or minimum value for ramp down. At min, 15% of
max speed. if (1.0 < RampDown || drivetype == false)
    {
      RampDown = 1.0;
    }

    // figure out how close is x to on line
    YCompleationRatio = (1 - fabs(y - CurrentYAxis) / fabs(y - initYLoc)) * 100;

    // Calculate Scale for hitting x dest., then hitting y dest.
    if (fabs((x - CurrentXAxis)) < fabs((TangentialDist)) && fabs((y -
CurrentYAxis)) < fabs((TangentialDist)))
    {
      DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((TangentialDist)));
      DriveXScale = YCompleationRatio * (fabs((x - CurrentXAxis)) /
fabs((TangentialDist))); DriveRScale = 50.0;
    }
    else
    {
      if (fabs((x - CurrentXAxis)) < fabs((y - CurrentYAxis)))
      {
        DriveYScale = 100.0;
        DriveXScale = YCompleationRatio * (fabs((x - CurrentXAxis)) / fabs((y -
CurrentYAxis))); DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((y -
CurrentYAxis)));
      }
      else
      {
        DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((x -
CurrentXAxis))); DriveXScale = YCompleationRatio; DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((x - CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((x - CurrentXAxis)) < 10.0 + increasediffs)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (x - CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((y - CurrentYAxis)) < 10.0 + increasediffs)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (y - CurrentYAxis < 0.0)
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

void Ram(double x, double y, double r)
{
  extern double CurrentXAxis;
  extern double CurrentYAxis;
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

  while (!((fabs((y - CurrentYAxis)) < 50.0 && fabs((TangentialDist)) < 50.0) &&
fabs((x - CurrentXAxis)) < 50.0))
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

    // Gives Time to ramp up speed before collision check regardless of previous
speed. HardCurve = HardCurve * 1.06; if (1.0 < HardCurve)
    {
      HardCurve = 1.0;
    }

    // Use quadratic functiion for ramp down
    RampDown = ((fabs((x - CurrentXAxis)) + fabs((y - CurrentYAxis))) + 1.5 *
fabs((TangentialDist))) * -0.015; RampDown = (RampDown * RampDown + 50.0) /
100.0; // Ramp down to at max 50% if (1.0 < RampDown)
    {
      RampDown = 1.0;
    }

    // Calculate Scale for direct line
    if (fabs((x - CurrentXAxis)) < fabs((TangentialDist)) && fabs((y -
CurrentYAxis)) < fabs((TangentialDist)))
    {
      DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((TangentialDist)));
      DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((TangentialDist)));
      DriveRScale = 50.0;
    }
    else
    {
      if (fabs((x - CurrentXAxis)) < fabs((y - CurrentYAxis)))
      {
        DriveYScale = 100.0;
        DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((y -
CurrentYAxis))); DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((y -
CurrentYAxis)));
      }
      else
      {
        DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((x -
CurrentXAxis))); DriveXScale = 100.0; DriveRScale = 50.0 *
(fabs((TangentialDist)) / fabs((x - CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((x - CurrentXAxis)) < 50.0)
    {
      DriveXDir = 0.0;
    }
    else
    {
      if (x - CurrentXAxis < 0.0)
      {
        DriveXDir = 1.0 * DriveXScale;
      }
      else
      {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((y - CurrentYAxis)) < 50.0)
    {
      DriveYDir = 0.0;
    }
    else
    {
      if (y - CurrentYAxis < 0.0)
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