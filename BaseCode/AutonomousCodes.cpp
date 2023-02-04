#include "AuxiliaryFunctions\Endgame.cpp"
#include "AuxiliaryFunctions\Flywheel.cpp"
#include "AuxiliaryFunctions\Intake.cpp"
#include "AuxiliaryFunctions\Roller.cpp"
#include "VectorEngine\Engine.cpp"
#include "vex.h"

/*
0 = odometer overwrite (x,y,w) write style 0 = overwrite, 1 = add
1 = simple destination(x,y,r,s)
2 = intake(v) velocity
3 = flywheel(v)
4 = trigger(p) pulse count
5 = wait(s) seconds
6 = roller(0,r) degrees (1,v) velocity
*/
/*double test_route[][5] = {{0, 230, 230, 180, 0},
                          {1, 600, 600, 90, 0.5},
                          //{1, -200, 300, 270, 0.7},
                          {1, 230, 230, 180, 0.5}};
int test_route_length = sizeof(test_route) / sizeof(double) / 5;*/

double flbleftdiskandrollerup[][5] = {
    {0, -814, 230, 270, 0},      // set origin
    {6, 0, 180, 0, 0},           // spin roller
    {3, 73, 0, 0, 0},            // spin up flywheel
    {1, -1900, 1220, 243.5, .7}, // move to middle of court
    {4, 4, 0, 0, 0},             // shoot two disks
    {3, 0, 0, 0, 0}              // spindown flywheel
};

const int flbleftdiskandroller_length = sizeof(flbleftdiskandrollerup) /
                                        sizeof(double) /
                                        5; // calculate length of array

double flbleftdiskandrollerdown[][5] = {
    {0, -814, 230, 270, 0},      // set origin
    {6, 0, -180, 0, 0},          // spin roller
    {3, 73, 0, 0, 0},            // spin up flywheel
    {1, -1900, 1220, 243.5, .7}, // move to middle of court
    {4, 4, 0, 0, 0},             // shoot two disks
    {3, 0, 0, 0, 0}              // spindown flywheel
};

const int flbleftdiskandrollerdown_length = sizeof(flbleftdiskandrollerdown) /
                                            sizeof(double) /
                                            5; // calculate length of array

double flbleftrollerup[][5] = {{0, 814, 230, 90, 0}, // set origin
                               {6, 0, 180, 0, 0}};

const int flbleftrollerup_length = sizeof(flbleftrollerup) / sizeof(double) / 5;

double flbleftrollerdown[][5] = {{0, 814, 230, 90, 0}, // set origin
                                 {6, 0, -180, 0, 0}};
const int flbleftrollerdown_length =
    sizeof(flbleftrollerdown) / sizeof(double) / 5; // calculate length of array

void AutonomousIndexer(double routine[][5], int length) {
  extern Robot_Telemetry ricky;
  for (int i = 0; i < length; i++) {
    if (routine[i][0] == 0) {   // odometer overwrite
      if (routine[i][4] == 0) { // write style 0 = overwrite, 1 = add
        ricky.CurrentXAxis = routine[i][1];
        ricky.TargetXAxis = routine[i][1];
        ricky.CurrentYAxis = routine[i][2];
        ricky.TargetYAxis = routine[i][2];
        Gyroscope.setHeading(routine[i][3], degrees);
        ricky.TargetTheta = routine[i][3];
      } else if (routine[i][4] == 1) {
        ricky.CurrentXAxis += routine[i][1];
        ricky.TargetXAxis += routine[i][1];
        ricky.CurrentYAxis += routine[i][2];
        ricky.TargetYAxis += routine[i][2];
        Gyroscope.setHeading(routine[i][3], degrees);
        ricky.TargetTheta += routine[i][3];
      }
    }
    if (routine[i][0] == 1) { // simple destination
      ricky.TargetXAxis = routine[i][1];
      ricky.TargetYAxis = routine[i][2];
      ricky.TargetTheta = routine[i][3];
      ricky.TargetSpeed = routine[i][4];
      ricky.StartXAxis = ricky.CurrentXAxis;
      ricky.StartYAxis = ricky.CurrentYAxis;
      ricky.StartTheta = ricky.CurrentThetaValue;
      vex::task::sleep(250);
      while (
          ricky.EngineBusy &&
          !ricky.TravelImpeded) { // wait for the engine to finish or get stuck
        vex::task::sleep(50);
      }
    } else if (routine[i][0] == 2) { // set intake velocity
      IntakeVelocity(routine[i][1]);
    } else if (routine[i][0] == 3) { // set flywheel velocity
      FlywheelVelocity(routine[i][1]);
    } else if (routine[i][0] == 4) { // trigger pulse x times
      TriggerPulse(int(routine[i][1]));
    } else if (routine[i][0] == 5) { // wait for x seconds
      vex::task::sleep(int(routine[i][1] * 1000));
    } else if (routine[i][0] == 6) { // roller
      if (routine[i][1] == 0) {
        Roller.setVelocity(50, percent);
        Roller.spinFor(forward, routine[i][2], degrees, true);
      } else if (routine[i][1] == 1) {
        Roller.setVelocity(routine[i][2], percent);
        Roller.spin(forward);
      }
    }
  }
}

int AutonomousRoutineDeamon() { // Main engine loop
  extern int test_route_length;
  AutonomousIndexer(flbleftrollerdown,
                    flbleftrollerdown_length); // runs through the given routine
  return 1;
};
