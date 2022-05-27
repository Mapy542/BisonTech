#include "vex.h"
#include "Odometry.cpp"
#include "MotorSigma.cpp"
#include "AuxiliaryFunctions/Telemetry.cpp"

//Precision for stopping at a place
void Destination(double x, double y, double r, double speed) {
  extern double CurrentXAxis;
  extern double CurrentYAxis;
  
  float DriveYScale = 0;
  float DriveXScale = 0;
  float DriveRScale = 0;
  float DriveYDir = 0;
  float DriveXDir = 0;
  float DriveYPower = 0;
  float DriveXPower = 0;
  float DriveRPower = 0;

  float RampUp = 0.01;
  float RampDown = 1.0;
  
  double TangentialDist = 2.0 * (FromGyro(r) * 6.2831);
  
  if (0.15 < FL.velocity(percent) || (0.15 < RL.velocity(percent) || (0.15 < FR.velocity(percent) || 0.15 < RR.velocity(percent)))) {
    RampUp = 1.0;
  }
  
  while (!((fabs((y - CurrentYAxis)) < 10.0 && fabs((TangentialDist)) < 10.0) && fabs((x - CurrentXAxis)) < 10.0)) {
    
    // Calculate Odometry
    Poll_Absolute_Cords();
    TangentialDist = 2.0 * (FromGyro(r) * 6.2831);
    
    // Calculate Ramp Powers
    RampUp = RampUp * 1.06;
    if (1.0 < RampUp) {
      RampUp = 1.0;
    }
    // Use quadratic functiion for ramp down
    RampDown = ((fabs((x - CurrentXAxis)) + fabs((y - CurrentYAxis))) + 1.5 * fabs((TangentialDist))) * -0.015;
    RampDown = (RampDown * RampDown + 15.0) / 100.0;
    if (1.0 < RampDown) {
      RampDown = 1.0;
    }

    // Calculate Scale for direct line
    if (fabs((x - CurrentXAxis)) < fabs((TangentialDist)) && fabs((y - CurrentYAxis)) < fabs((TangentialDist))) {
      DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((TangentialDist)));
      DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((TangentialDist)));
      DriveRScale = 50.0;
    }
    else {
      if (fabs((x - CurrentXAxis)) < fabs((y - CurrentYAxis))) {
        DriveYScale = 100.0;
        DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((y - CurrentYAxis)));
        DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((y - CurrentYAxis)));
      }
      else {
        DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((x - CurrentXAxis)));
        DriveXScale = 100.0;
        DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((x - CurrentXAxis)));
      }
    }

    // Solve for direction
    if (fabs((x - CurrentXAxis)) < 10.0) {
      DriveXDir = 0.0;
    }
    else {
      if (x - CurrentXAxis < 0.0) {
        DriveXDir = 1.0 * DriveXScale;
      }
      else {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((y - CurrentYAxis)) < 10.0) {
      DriveYDir = 0.0;
    }
    else {
      if (y - CurrentYAxis < 0.0) {
        DriveYDir = 1.0 * DriveYScale;
      }
      else {
        DriveYDir = -1.0 * DriveYScale;
      }
    }
    if (fabs((TangentialDist)) < 10.0) {
      DriveRPower = 0.0;
    }
    else {
      if (TangentialDist < 0.0) {
        DriveRPower = 1.0 * DriveRScale;
      }
      else {
        DriveRPower = -1.0 * DriveRScale;
      }
    }

    // Calculate motor powers over trig values
    DriveXPower = (DriveYDir * sin(Gyroscope.heading(degrees) * M_PI / 180) + (DriveXDir * cos(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0) * 1.0;
    DriveYPower = (DriveYDir * cos(Gyroscope.heading(degrees) * M_PI / 180) + DriveXDir * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 1.0;
    // Apply motor powers to individual motors
    DriveMotors(DriveXPower, DriveYPower, DriveRPower, speed * (RampUp * RampDown));
    //Print_XYR();
  wait(5, msec);
  }
  DriveMotors(0.0, 0.0, 0.0, 0.0);
}

//Speedy for navigating around obstacles
void Waypoint(double x, double y, double r, double speed) {
  extern double CurrentXAxis;
  extern double CurrentYAxis;
  
  float DriveYScale = 0;
  float DriveXScale = 0;
  float DriveRScale = 0;
  float DriveYDir = 0;
  float DriveXDir = 0;
  float DriveYPower = 0;
  float DriveXPower = 0;
  float DriveRPower = 0;

  float RampUp = 0.01;
  
  double TangentialDist = 2.0 * (FromGyro(r) * 6.2831);
  if (0.15 < FL.velocity(percent) || (0.15 < RL.velocity(percent) || (0.15 < FR.velocity(percent) || 0.15 < RR.velocity(percent)))) {
    RampUp = 1.0;
  }

  while (!((fabs((y - CurrentYAxis)) < 50.0 && fabs((TangentialDist)) < 50.0) && fabs((x - CurrentXAxis)) < 50.0)) {
    
    // Calculate Odometry
    Poll_Absolute_Cords();
    TangentialDist = 2.0 * (FromGyro(r) * 6.2831);
    
    // Calculate Ramp Powers
    RampUp = RampUp * 1.06;
    if (1.0 < RampUp) {
      RampUp = 1.0;
    }
    
    // Calculate Scale for direct line
    if (fabs((x - CurrentXAxis)) < fabs((TangentialDist)) && fabs((y - CurrentYAxis)) < fabs((TangentialDist))) {
      DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((TangentialDist)));
      DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((TangentialDist)));
      DriveRScale = 50.0;
    }
    else {
      if (fabs((x - CurrentXAxis)) < fabs((y - CurrentYAxis))) {
        DriveYScale = 100.0;
        DriveXScale = 100.0 * (fabs((x - CurrentXAxis)) / fabs((y - CurrentYAxis)));
        DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((y - CurrentYAxis)));
      }
      else {
        DriveYScale = 100.0 * (fabs((y - CurrentYAxis)) / fabs((x - CurrentXAxis)));
        DriveXScale = 100.0;
        DriveRScale = 50.0 * (fabs((TangentialDist)) / fabs((x - CurrentXAxis)));
      }
    }
    
    // Solve for direction
    if (fabs((x - CurrentXAxis)) < 50.0) {
      DriveXDir = 0.0;
    }
    else {
      if (x - CurrentXAxis < 0.0) {
        DriveXDir = 1.0 * DriveXScale;
      }
      else {
        DriveXDir = -1.0 * DriveXScale;
      }
    }
    if (fabs((y - CurrentYAxis)) < 50.0) {
      DriveYDir = 0.0;
    }
    else {
      if (y - CurrentYAxis < 0.0) {
        DriveYDir = 1.0 * DriveYScale;
      }
      else {
        DriveYDir = -1.0 * DriveYScale;
      }
    }
    if (fabs((TangentialDist)) < 50.0) {
      DriveRPower = 0.0;
    }
    else {
      if (TangentialDist < 0.0) {
        DriveRPower = 1.0 * DriveRScale;
      }
      else {
        DriveRPower = -1.0 * DriveRScale;
      }
    }
    
    // Calculate motor powers over trig values
    DriveXPower = (DriveYDir * sin(Gyroscope.heading(degrees) * M_PI / 180) + (DriveXDir * cos(Gyroscope.heading(degrees) * M_PI / 180)) * -1.0) * 1.0;
    DriveYPower = (DriveYDir * cos(Gyroscope.heading(degrees) * M_PI / 180) + DriveXDir * sin(Gyroscope.heading(degrees) * M_PI / 180)) * 1.0;
    // Apply motor powers to individual motors
    DriveMotors(DriveXPower, DriveYPower, DriveRPower, speed * (RampUp * 1.0));
    //Print_XYR();
  wait(5, msec);
  }
  DriveMotors(0.0, 0.0, 0.0, 0.0);
}