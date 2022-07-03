#include "vex.h"
#include "Robot_Telemetry_Structure.cpp"

void Odometry_Daemon{
  extern Robot_Telemetry ricky;

  while(true)
  {
    // Update odometry
    EncoderIntegral();
    vex::task::sleep(10);
  }
}

void EncoderIntegral(){ //Update odometry from encoders by integrating encoder values
  extern Robot_Telemetry ricky; //Forces update every time
  double DeltaTheta;
  double EncoderDeltaY;
  double EncoderDeltaX;
  double DeltaTime;

  DeltaTheta = Gyroscope.heading(degrees) - ricky.CurrentThetaValue; //Calculate changes from previous cycle
  EncoderDeltaY = y.rotation(degrees) - ricky.CurrentYEncoderValue;
  EncoderDeltaX = x.rotation(degrees) - ricky.CurrentXEncoderValue;
  DeltaTime = (double)vex::timer::system() - ricky.CurrentTime; //Calculate time difference

  ricky.CurrentXEncoderValue = x.rotation(degrees); //Mark down current encoder values for reference next cycle
  ricky.CurrentYEncoderValue = y.rotation(degrees);
  ricky.CurrentThetaValue = Gyroscope.heading(degrees);
  ricky.CurrentTime = vex::timer::system();

  /*
  // Dont do this but checks for 360 deg wrap
  // then gives delta theta an average change ammount.
  // may cause slight error
  if (DeltaTheta > 300.0) {
    DeltaTheta = 0.5;
  }
  if (-300.0 > DeltaTheta) {
    DeltaTheta = -0.5;
  }*/

  if(abs(DeltaTheta) > 300){ //If heading wraps around 360 degrees, then guess theta based on current speed
    DeltaTheta = ricky.CurrentRVelocity * .7; //.7 is a guess of how much theta changes per rotation
  }

  //Integrate encoder values to get odometry
  double XChange = ((EncoderDeltaY * sin(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0 + (EncoderDeltaX - DeltaTheta * -1.60520825) * cos(Gyroscope.heading(degrees) * M_PI / 180)) * 0.620639082;
  double YChange = ((EncoderDeltaY * cos(Gyroscope.heading(degrees) * M_PI / 180) + (EncoderDeltaX - DeltaTheta * -1.60520825) * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 0.70639082) * -1.0;
  ricky.CurrentXAxis += XChange;
  ricky.CurrentYAxis += YChange;

  //Calculate current velocitys
  ricky.CurrentXVelocity = XChange / (double)DeltaTime / 1000; //mm/s
  ricky.CurrentYVelocity = YChange / (double)DeltaTime / 1000; //mm/s
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