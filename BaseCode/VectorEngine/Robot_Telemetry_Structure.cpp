typedef struct{ //a javalike classlike thingy but just with a bunch of public fields

int TunedMass; //Used for Odometry and kinematics

double CurrentXAxis; //From Odometry systems
double CurrentYAxis;

double TargetXAxis; //For vectoring engine
double TargetYAxis;
double TargetTheta;

double OverideXAxis; //For mutable object positions
double OverideYAxis;
double OverideTheta;

double CurrentXEncoderValue;//Used for encoder odemetry integration(calculus)
double CurrentYEncoderValue;
double CurrentThetaValue;

double CurrentXVelocity;//Used for vectoring engine
double CurrentYVelocity;
double CurrentRVelocity;

double MaxXVelocity = 1;//Vectoring constants
double MaxYVelocity = 1;
double MaxRVelocity = 1;

double CurrentXAcceleration;//Used for vectoring engine
double CurrentYAcceleration;
double CurrentRAcceleration;

double MaxXAcceleration = 1;//Vectoring constants
double MaxYAcceleration = 1;
double MaxRAcceleration = 1;

long CurrentTime;//Used for odometry integration

}Robot_Telemetry;


void PrintTelemetry(Robot_Telemetry robo) {
  printf("\n");
  printf("%.6f", "CurrentXAxis" + static_cast<float>(robo.CurrentXAxis) + "\n");
    printf("%.6f", "CurrentYAxis" + static_cast<float>(robo.CurrentYAxis) + "\n");
    printf("%.6f", "CurrentThetaValue" + static_cast<float>(robo.CurrentThetaValue) + "\n");
    printf("%.6f", "CurrentXVelocity" + static_cast<float>(robo.CurrentXVelocity) + "\n");
    printf("%.6f", "CurrentYVelocity" + static_cast<float>(robo.CurrentYVelocity) + "\n");
    printf("%.6f", "CurrentRVelocity" + static_cast<float>(robo.CurrentRVelocity) + "\n");
    printf("%.6f", "CurrentXAcceleration" + static_cast<float>(robo.CurrentXAcceleration) + "\n");
    printf("%.6f", "CurrentYAcceleration" + static_cast<float>(robo.CurrentYAcceleration) + "\n");
    printf("%.6f", "CurrentRAcceleration" + static_cast<float>(robo.CurrentRAcceleration) + "\n");
    printf("%.6f", "CurrentTime" + static_cast<float>(robo.CurrentTime) + "\n");
    printf("%.6f", "TunedMass" + static_cast<float>(robo.TunedMass) + "\n");
    printf("%.6f", "TargetXAxis" + static_cast<float>(robo.TargetXAxis) + "\n");
    printf("%.6f", "TargetYAxis" + static_cast<float>(robo.TargetYAxis) + "\n");
    printf("%.6f", "TargetTheta" + static_cast<float>(robo.TargetTheta) + "\n");
    printf("%.6f", "OverideXAxis" + static_cast<float>(robo.OverideXAxis) + "\n");
    printf("%.6f", "OverideYAxis" + static_cast<float>(robo.OverideYAxis) + "\n");
    printf("%.6f", "OverideTheta" + static_cast<float>(robo.OverideTheta) + "\n");
    printf("%.6f", "CurrentXEncoderValue" + static_cast<float>(robo.CurrentXEncoderValue) + "\n");
    printf("%.6f", "CurrentYEncoderValue" + static_cast<float>(robo.CurrentYEncoderValue) + "\n");
}