#include "vex.h"
#include <cmath>
// Polar Translation from cartesian to polar coordinates and back after rotation
// transformation

double ToPolarMagnitude(double x, double y) {
  double polarM = sqrt(x * x + y * y); // Convert magnitude
  return polarM;
}

double ToPolarAngle(double x, double y) {
  double polarTheta = atan2(y, x) * 180.0 /
                      M_PI; // Convert angle then back to degrees cause why
                            // would cpp include degree functions by default.
  return polarTheta;
}

double ToCartesianX(double magnitude, double angle) {
  double cartesianX =
      magnitude * cos(angle * M_PI / 180); // Convert cartesian x
  return cartesianX;
}

double ToCartesianY(double magnitude, double angle) {
  double cartesianY =
      magnitude * sin(angle * M_PI / 180); // Convert cartesian y
  return cartesianY;
}

void PolarTransformation(double x, double y,
                         double theta) { // Rotate a vector by theta degrees
  extern Robot_Telemetry ricky;
  double polarM = ToPolarMagnitude(x, y); // Convert to polar
  double polarTheta = ToPolarAngle(x, y) + theta;
  ricky.TransformReturnX = ToCartesianX(polarM, polarTheta); // Convert back to
                                                             // cartesian
  ricky.TransformReturnY = ToCartesianY(polarM, polarTheta);
}
void EnginePolarTransformation(
    double x, double y,
    double theta) { // Rotate a vector by theta degrees
  extern Robot_Telemetry ricky;
  double polarM = ToPolarMagnitude(x, y); // Convert to polar
  double polarTheta = ToPolarAngle(x, y) + theta;
  ricky.EngineTransformReturnX =
      ToCartesianX(polarM, polarTheta); // Convert back to
                                        // cartesian
  ricky.EngineTransformReturnY = ToCartesianY(polarM, polarTheta);
}