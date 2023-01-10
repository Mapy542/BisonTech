#include "Robot_Telemetry_Structure.cpp"
#include "vex.h"

void EncoderIntegral() { // Update odometry from encoders by integrating encoder
                         // values
  extern Robot_Telemetry ricky; // Forces update every time
  double DeltaTheta;
  double EncoderDeltaY;
  double EncoderDeltaX;
  double DeltaTime;

  DeltaTheta = Gyroscope.heading(degrees) -
               ricky.CurrentThetaValue; // Calculate changes from previous cycle
  EncoderDeltaY = y.rotation(degrees) - ricky.CurrentYEncoderValue;
  EncoderDeltaX = x.rotation(degrees) - ricky.CurrentXEncoderValue;
  DeltaTime = (double)vex::timer::system() -
              ricky.CurrentTime; // Calculate time difference

  ricky.CurrentXEncoderValue = x.rotation(
      degrees); // Mark down current encoder values for reference next cycle
  ricky.CurrentYEncoderValue = y.rotation(degrees);
  ricky.CurrentThetaValue = Gyroscope.heading(degrees);
  ricky.CurrentTime = vex::timer::system();

  /*// Dont do this but checks for 360 deg wrap
  // then gives delta theta an average change amount.
  // may cause slight error
  if (DeltaTheta > 300.0) {
    DeltaTheta = 0.5;
  }
  if (-300.0 > DeltaTheta) {
    DeltaTheta = -0.5;
  }
  */

  if (fabs(DeltaTheta) > 300) { // If heading wraps around 360 degrees, then
                                // guess theta based on current speed
    DeltaTheta = ricky.CurrentRVelocity *
                 .7; //.7 is a guess of how much theta changes per rotation
  }

  // Integrate encoder values to get odometry
  double XChange = (EncoderDeltaY - DeltaTheta * ricky.YEncoderError) *
                       sin(Gyroscope.heading(degrees) * M_PI / 180) +
                   (EncoderDeltaX - DeltaTheta * ricky.XEncoderError) *
                       cos(Gyroscope.heading(degrees) * M_PI / 180) *
                       ricky.EncoderTicksPerMM;
  double YChange = (EncoderDeltaY - DeltaTheta * ricky.YEncoderError) *
                       cos(Gyroscope.heading(degrees) * M_PI / 180) +
                   (EncoderDeltaX - DeltaTheta * ricky.XEncoderError) *
                       sin(Gyroscope.heading(degrees) * M_PI / 180) *
                       ricky.EncoderTicksPerMM;

  ricky.CurrentXAxis += XChange; // accumulate change in position
  ricky.CurrentYAxis += YChange;

  // derive velocity from change in position over time
  ricky.CurrentXVelocity = XChange / (double)DeltaTime;    // mm/ms
  ricky.CurrentYVelocity = YChange / (double)DeltaTime;    // mm/ms
  ricky.CurrentRVelocity = DeltaTheta / (double)DeltaTime; // deg/ms
};