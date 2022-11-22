typedef struct { // a javalike classlike thingy but just with a bunch of public
                 // fields

  double TunedMass; // Used for Odometry and kinematics

  double CurrentXAxis; // From Odometry systems
  double CurrentYAxis;

  double TargetXAxis; // For vectoring engine
  double TargetYAxis;
  double TargetTheta;

  double OverideXAxis; // For mutable object positions
  double OverideYAxis;
  double OverideTheta;

  double CurrentXEncoderValue; // Used for encoder odemetry
                               // integration(calculus)
  double CurrentYEncoderValue;
  double CurrentThetaValue;

  double CurrentXVelocity; // Used for vectoring engine
  double CurrentYVelocity;
  double CurrentRVelocity;

  // double MaxXVelocity = 1;//Vectoring constants
  // double MaxYVelocity = 1;
  // double MaxRVelocity = 1;

  double CurrentXAcceleration; // Used for vectoring engine
  double CurrentYAcceleration;
  double CurrentRAcceleration;

  // double MaxXAcceleration = 1;//Vectoring constants
  // double MaxYAcceleration = 1;
  // double MaxRAcceleration = 1;

  long CurrentTime; // Used for odometry integration

  double Override_R_Speed; // Used for manual control
  bool Override_Manual_R;  // Used for manual control
} Robot_Telemetry;

/*void PrintTelemetry(Robot_Telemetry robo) {
  printf("\n");
  printf("%.6f", "CurrentXAxis" + < (robo.CurrentXAxis) + "\n");
}*/