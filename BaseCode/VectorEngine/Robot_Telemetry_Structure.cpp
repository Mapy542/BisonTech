#include "vex.h"
#ifndef Robot_Telemetry_Structure_cpp_ // ensure that this file is only
                                       // included once
#define Robot_Telemetry_Structure_cpp_

struct Robot_Telemetry { // a java like class like thingy but just with
                         // a bunch of
                         // public fields
  // CONSTANTS////////////////////////////////////////////////
  const double MaxAcceleration = 100; // percent per cycle

  const double XEncoderError =
      0; // 1.60520825; // Used for encoder angular error correction
  const double YEncoderError =
      0.575; // Used for encoder angular error correction

  const double EncoderTicksPerMM = 0.6223678817; // mm per deg

  const float DistanceTolerance = 5; // mm
  const float AngleTolerance = 2;    // degrees

  // ODOMETRY//////////////////////////////////////////////////////////
  double CurrentXAxis = 0; // From Odometry systems
  double CurrentYAxis = 0;

  double CurrentXEncoderValue = 0; // Used for encoder odometry
                                   // integration(calculus)
  double CurrentYEncoderValue = 0;
  double CurrentThetaValue = 270;

  double CurrentXVelocity = 0; // Used for vectoring engine
  double CurrentYVelocity = 0;
  double CurrentRVelocity = 0;

  long CurrentTime = 0; // Used for odometry integration

  // MANUAL HYBRID CONTROL///////////////////////////////////////////////
  double Override_R_Speed;        // Used for manual control
  bool Override_Manual_R = false; // Used for manual control

  // ENGINE////////////////////////////////////////////////////////

  bool EngineBusy = false;    // flag for if the engine is busy
  bool TravelImpeded = false; // flag for if the robot is stuck

  double TargetXVelocity = 0; // Used for vectoring engine
  double TargetYVelocity = 0; // mm/ms
  double TargetRVelocity = 0;
  double TargetTotalVelocity = 0;

  double SetXVelocity = 0; // Used for calculating velocity change
  double SetYVelocity = 0;
  double SetRVelocity = 0;

  long BusyStartTime; // Used for acceleration and stuck detection

  // PATHING////////////////////////////////////////////////////////

  double TargetXAxis = 0; // Used to describe next waypoint
  double TargetYAxis = 0;
  double TargetTheta = 180;
  double TargetSpeed = 1;

  double StartXAxis = 0; // Used to describe current waypoint
  double StartYAxis = 0;
  double StartTheta = 180;

  bool Targeting = false; // flag for if the robot must prioritize rotation to a
                          // target

  bool AutoDone = false; // flag for if the robot is done with the routine

  // POLAR TRANSFORMATION///////////////////////////////////////////////

  double TransformReturnX; // Used for polar transformation
  double TransformReturnY; // easier to return in the struc than to pass by
                           // reference

  // Location Power
  // Flywheel////////////////////////////////////////////////////////////

  int FlywheelTargetVelocity = 0; // percent

  // fast changing PV should have a high Ki and low Kp
  const float FlywheelLow[3] = {2.4, 0.6, 0.3}; // [P, I, D]
  const float FlywheelHigh[3] = {0.8, 1, 0.7};  // [P, I, D]

  int FlywheelLastError = 0;  // Used for PID control
  int FlywheelTotalError = 0; // Used for PID control

  const float FlywheelMin = 0.4;        // flywheel min speed
  const float FlywheelMax = 1.0;        // flywheel max speed
  const int GoalXPosition = -18 * 25.4; // 18 inches
  const int GoalYPosition = 122 * 25.4; // 10' 3"
  const int MaximumFlywheelDistance =
      140 * 25.4; //  12' 0" maximum possible distance from goal

  const int LauncherAngleCompensation = 10; // 0 degrees
};

#endif // end double define check