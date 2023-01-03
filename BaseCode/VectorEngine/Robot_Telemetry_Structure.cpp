#ifndef Robot_Telemetry_Structure_cpp_ // ensure that this file is only
                                       // included once
#define Robot_Telemetry_Structure_cpp_

struct Robot_Telemetry { // a java like class like thingy but just with
                         // a bunch of
                         // public fields

  // CONSTANTS////////////////////////////////////////////////
  const double MaxAcceleration = 1; // percent

  const double XEncoderError = 0; // Used for encoder angular error correction
  const double YEncoderError = 0; // Used for encoder angular error correction

  const double EncoderTicksPerMM = 0.6223678817; // mm per deg

  const float DistanceTolerance = 4; // mm
  const float AngleTolerance = 2;    // degrees

  const double MaxRobotVelocity =
      600 / 2 / EncoderTicksPerMM; // motors at 600 rpm / 2 because 45 deg then
                                   // convert deg to mm theoretically 500
                                   // mm/m???? might be bad
  // ODOMETRY//////////////////////////////////////////////////////////
  double CurrentXAxis = 0; // From Odometry systems
  double CurrentYAxis = 0;

  double CurrentXEncoderValue = 0; // Used for encoder odometry
                                   // integration(calculus)
  double CurrentYEncoderValue = 0;
  double CurrentThetaValue = 180;

  double CurrentXVelocity = 0; // Used for vectoring engine
  double CurrentYVelocity = 0;
  double CurrentRVelocity = 0;

  long CurrentTime = 0; // Used for odometry integration

  // MANUAL HYBRID CONTROL///////////////////////////////////////////////
  double Override_R_Speed;        // Used for manual control
  bool Override_Manual_R = false; // Used for manual control

  // ENGINE////////////////////////////////////////////////////////

  bool Engine_Busy = false;    // flag for if the engine is busy
  bool Travel_Impeded = false; // flag for if the robot is stuck

  int TravelStyle = 0; // 0 = direct line, 1 = arc x first, 2 = arc y first

  double TargetXAxis = 0; // Used to describe next waypoint
  double TargetYAxis = 0;
  double TargetTheta = 180;
  double TargetSpeed = 1;

  double FutureXAxis =
      0; // Used to describe next next waypoint for transition calculations
  double FutureYAxis = 0; // Set to same as Target waypoint if stop desired
  double FutureTheta = 180;
  double FutureSpeed = 1; // speed should be 0 if stop desired

  double StartingXAxis =
      0; // Used to describe start of travel for completion ratio
  double StartingYAxis = 0;
  double StartingTheta = 180;
  double StartingSpeed = 1;

  long Busy_Start_Time; // Used for acceleration and stuck detection
};

/*void PrintTelemetry(Robot_Telemetry robo) {
  printf("\n");
  printf("%.6f", "CurrentXAxis" + < (robo.CurrentXAxis) + "\n");
}*/

#endif // end double define check