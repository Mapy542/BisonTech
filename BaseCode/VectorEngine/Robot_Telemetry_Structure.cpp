typedef struct{ //a javalike classlike thingy but just with a bunch of public fields

int mass; //kg

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