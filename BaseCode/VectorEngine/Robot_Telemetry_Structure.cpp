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
      600 / 2 / EncoderTicksPerMM; // motors at 600 rpm / 2 becuase 45 deg then
                                   // convert deg to mm theoretically 500
                                   // mm/m???? might be bad
  // ODOMETRY//////////////////////////////////////////////////////////
  double CurrentXAxis; // From Odometry systems
  double CurrentYAxis;

  double TargetXAxis; // For vectoring engine
  double TargetYAxis;
  double TargetTheta;
  double TargetSpeed; // For vectoring engine

  double CurrentXEncoderValue; // Used for encoder odemetry
                               // integration(calculus)
  double CurrentYEncoderValue;
  double CurrentThetaValue;

  double CurrentXVelocity; // Used for vectoring engine
  double CurrentYVelocity;
  double CurrentRVelocity;

  long CurrentTime; // Used for odometry integration

  // MANUAL HYBRID CONTROL///////////////////////////////////////////////
  double Override_R_Speed; // Used for manual control
  bool Override_Manual_R;  // Used for manual control

  // ENGINE////////////////////////////////////////////////////////

  bool Engine_Busy; // flag for if the engine is busy

  int TravelStyle = 0; // 0 = direct line, 1 = arc x first, 2 = arc y first
};

/*void PrintTelemetry(Robot_Telemetry robo) {
  printf("\n");
  printf("%.6f", "CurrentXAxis" + < (robo.CurrentXAxis) + "\n");
}*/

#endif // end double define check