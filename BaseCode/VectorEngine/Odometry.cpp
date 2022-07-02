#include "vex.h"

void Odometry_Daemon{
  extern Robot_Telemetry ricky;

  while(true)
  {
    // Update odometry
    struct encoderoutput = EncoderIntegral(ricky);
    vex::task::sleep(10);
  }
}

void EncoderIntegral(struct robo){



  typedef struct{
    double x = a;
    double y = a;
  }point;
  return point;
}

//Get Odometry from encoders and gyroscope
//POLL AS FAST AS POSSIBLE
/* This is the function that polls the encoders and the gyroscope to get the absolute position of the
robot. */
void asdPoll_Absolute_Cords() {
  double DeltaTheta;
  double EncoderDeltaY;
  double EncoderDeltaX;
  extern double PreviousYValue;
  extern double PreviousXValue;
  extern double PreviousTheta;
  extern double CurrentXAxis;
  extern double CurrentYAxis;
  extern double globaldelta;

  DeltaTheta = Gyroscope.heading(degrees) - PreviousTheta;
  EncoderDeltaY = y.rotation(degrees) - PreviousYValue;
  EncoderDeltaX = x.rotation(degrees) - PreviousXValue;

  PreviousXValue = x.rotation(degrees);
  PreviousYValue = y.rotation(degrees);
  PreviousTheta = Gyroscope.heading(degrees);

  // Dont do this but checks for 360 deg wrap
  // then gives delta theta an average change ammount.
  // may cause slight error
  if (DeltaTheta > 300.0) {
    DeltaTheta = 0.5;
  }
  if (-300.0 > DeltaTheta) {
    DeltaTheta = -0.5;
  }

  globaldelta = abs(EncoderDeltaX) + fabs(EncoderDeltaY);
  CurrentXAxis = CurrentXAxis + ((EncoderDeltaY * sin(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0 + (EncoderDeltaX - DeltaTheta * -1.60520825) * cos(Gyroscope.heading(degrees) * M_PI / 180)) * 0.620639082;
  CurrentYAxis = CurrentYAxis + ((EncoderDeltaY * cos(Gyroscope.heading(degrees) * M_PI / 180) + (EncoderDeltaX - DeltaTheta * -1.60520825) * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 0.70639082) * -1.0;
}

//Just an easy function to move declare where the bot is.
/* This function sets the current position of the robot to the given x and y coordinates. */
void Set_Offset(double x, double y) {
  extern double CurrentXAxis;
  extern double CurrentYAxis;
  CurrentXAxis = x;
  CurrentYAxis = y;
}


//Easy calculate vector value to coord. Reports positive and negative for direction. Solves wrap around.
/* This function returns the difference between the gyroscope and the given value. If the difference is
greater than 180 degrees, it will return the difference minus 360 degrees. */
float FromGyro(double r) {
  double DegreeDiff = 0;
  if (fabs(Gyroscope.heading(degrees) - r) > 180.0) {
    DegreeDiff = ((Gyroscope.heading(degrees) - r) - 359.0) * -1.0;
  }
  else {
    DegreeDiff = Gyroscope.heading(degrees) - r;
  }
  return DegreeDiff;
}