#include "Robot_Telemetry_Structure.cpp"
#include "vex.h"
/*
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

  /*
  // Dont do this but checks for 360 deg wrap
  // then gives delta theta an average change ammount.
  // may cause slight error
  if (DeltaTheta > 300.0) {
    DeltaTheta = 0.5;
  }
  if (-300.0 > DeltaTheta) {
    DeltaTheta = -0.5;
  }

  if (fabs(DeltaTheta) > 300) { // If heading wraps around 360 degrees, then
                                // guess theta based on current speed
    DeltaTheta = ricky.CurrentRVelocity *
                 .7; //.7 is a guess of how much theta changes per rotation
  }

  // Integrate encoder values to get odometry
  double XChange =
      ((EncoderDeltaY * sin(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0 +
       (EncoderDeltaX - DeltaTheta * -1.60520825) *
           cos(Gyroscope.heading(degrees) * M_PI / 180)) *
      0.6223678817;
  double YChange =
      ((EncoderDeltaY * cos(Gyroscope.heading(degrees) * M_PI / 180) +
        (EncoderDeltaX - DeltaTheta * -1.60520825) *
            sin(Gyroscope.heading(degrees) * M_PI / 180)) *
       0.6223678817) *
      -1.0;
  ricky.CurrentXAxis += XChange;
  ricky.CurrentYAxis += YChange;

  // Calculate current velocitys

  // integration method:
  // v = x/t
  double XVelDeltaTime = XChange / (double)DeltaTime / 1000;    // mm/s
  double YVelDeltaTime = YChange / (double)DeltaTime / 1000;    // mm/s
  double RVelDeltaTime = DeltaTheta / (double)DeltaTime / 1000; // deg/s

  // hardware dps method
  // v = v
  double XVelDPS =
      ((y.velocity(dps) * sin(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0 +
       (x.velocity(dps)) * cos(Gyroscope.heading(degrees) * M_PI / 180)) *
      0.6223678817;
  double YVelDPS =
      ((y.velocity(dps) * cos(Gyroscope.heading(degrees) * M_PI / 180) +
        (x.velocity(dps)) * sin(Gyroscope.heading(degrees) * M_PI / 180)) *
       0.6223678817) *
      -1.0;
  double RVelDPS = Gyroscope.gyroRate(xaxis, dps);

  // motors telemetry method:  (may have issues if the motors are slipping)
  // v = v
  double forwardvelocity = (FL.velocity(dps) + FR.velocity(dps) +
                            RL.velocity(dps) + RR.velocity(dps)) /
                           4 * 0.8890969738; // mm/s
  double sidewardvelocity = (-1 * FL.velocity(dps) + FR.velocity(dps) +
                             RL.velocity(dps) - RR.velocity(dps)) /
                            4 * 0.8890969738; // mm/s
  double rotationalvelocity = (FL.velocity(dps) + FR.velocity(dps) -
                               RL.velocity(dps) - RR.velocity(dps)) /
                              4; // deg/s

  // Average possible velocitys (less emphasis on motors)
  ricky.CurrentXVelocity =
      (XVelDeltaTime + XVelDPS + sidewardvelocity / 2) / 2.5; // mm/s
  ricky.CurrentYVelocity =
      (YVelDeltaTime + YVelDPS + forwardvelocity / 2) / 2.5; // mm/s
  ricky.CurrentRVelocity =
      (RVelDeltaTime + RVelDPS + rotationalvelocity / 2) / 2.5; // deg/s

  // Calculate current accelerations via double integration
  // a = v/t
  double XAclDeltaTime =
      ricky.CurrentXVelocity / (double)DeltaTime / 1000; // mm/s^2
  double YAclDeltaTime =
      ricky.CurrentYVelocity / (double)DeltaTime / 1000; // mm/s^2
  double RAclDeltaTime =
      ricky.CurrentRVelocity / (double)DeltaTime / 1000; // deg/s^2

  // hardware dps method:
  // a = v/t
  double XAclhardware = sidewardvelocity / (double)DeltaTime / 1000; // mm/s^2
  double YAclhardware = forwardvelocity / (double)DeltaTime / 1000;  // mm/s^2
  double RAclhardware =
      rotationalvelocity / (double)DeltaTime / 1000; // deg/s^2

  // Average possible accelerations (less emphasis on motors)
  ricky.CurrentXAcceleration =
      (XAclDeltaTime + XAclhardware / 2) / 1.5; // mm/s^2
  ricky.CurrentYAcceleration =
      (YAclDeltaTime + YAclhardware / 2) / 1.5; // mm/s^2
  ricky.CurrentRAcceleration =
      (RAclDeltaTime + RAclhardware / 2) / 1.5; // deg/s^2
};

// Precondition: function takes two rotations as arguments
// Postcondition: returns the difference between the two rotations while
// handling wrap around 360 degrees. Should return the shortest distance between
// the two rotations with sign denoting the direction. + = ccw, - = cw
double RadialDifference(double deg1, double deg2){};

// Precondition: Robot acceleration is assumed to be correct and uptodate
// Postcondition: Returns best guess of the robots mass based on the current
// acceleration and torque on the motors
double MassTune(){};

// Precondition: function is given a new value for a parameter
// Postcondition: if the new value is within a 20% deviation from the current
// value, the average of the two values is returned //if the current value is 0,
// the new value is returned
double Averager(double newval, double currentval){};

void Odometry_Daemon() { // Main odometry service loop
  // extern Robot_Telemetry ricky;
  while (true) {
    // Update odometry
    EncoderIntegral(); // Does 90% of the work
    // LidarUpdate(); //Double checks absolute distances to keep integral in
    // check when possible
    // PrintTelemetry(ricky);
    vex::task::sleep(10);
  }
};

*/