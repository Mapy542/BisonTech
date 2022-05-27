#include "vex.h"

//Get Odometry from encoders and gyroscope
//POLL AS FAST AS POSSIBLE
void Poll_Absolute_Cords() {
  double DeltaTheta;
  double EncoderDeltaY;
  double EncoderDeltaX;
  extern double PreviousYValue;
  extern double PreviousXValue;
  extern double PreviousTheta;
  extern double CurrentXAxis;
  extern double CurrentYAxis;

  DeltaTheta = Gyroscope.heading(degrees) - PreviousTheta;
  EncoderDeltaY = y.rotation(degrees) - PreviousYValue;
  EncoderDeltaX = (x.rotation(degrees) - 0.0) - PreviousXValue;

  PreviousXValue = x.rotation(degrees) - 0.0;
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

  CurrentXAxis = CurrentXAxis + ((EncoderDeltaY * sin(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0 + (EncoderDeltaX - DeltaTheta * -1.60520825) * cos(Gyroscope.heading(degrees) * M_PI / 180)) * 0.609;
  CurrentYAxis = CurrentYAxis + ((EncoderDeltaY * cos(Gyroscope.heading(degrees) * M_PI / 180) + (EncoderDeltaX - DeltaTheta * -1.60520825) * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 0.609) * -1.0;
}

//Just an easy function to move declare where the bot is.
void Set_Offset(double x, double y) {
  extern double CurrentXAxis;
  extern double CurrentYAxis;
  CurrentXAxis = x;
  CurrentYAxis = y;
}


//Easy calculate vector value to coord. Reports positive and negative for direction. Solves wrap around.
float FromGyro(double r) {
  double DegreeDiff = 0;
  if (fabs((Gyroscope.heading(degrees) - r)) > 180.0) {
    DegreeDiff = ((Gyroscope.heading(degrees) - r) - 360.0) * -1.0;
  }
  else {
    DegreeDiff = Gyroscope.heading(degrees) - r;
  }
  return DegreeDiff;
}