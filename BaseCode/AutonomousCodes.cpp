#include "AuxiliaryFunctions\Endgame.cpp"
#include "AuxiliaryFunctions\Flywheel.cpp"
#include "AuxiliaryFunctions\Intake.cpp"
#include "AuxiliaryFunctions\Roller.cpp"
#include "AuxiliaryFunctions\Vision.cpp"
#include "VectorEngine\Engine.cpp"
#include "vex.h"

/*
0 = odometer overwrite (x,y,w) write style 0 = overwrite, 1 = add
1 = simple destination(x,y,r,s)
7 = simple waypoint(x,y,r,s) no proximity ramping
2 = intake(v) velocity
3 = flywheel(v)
4 = trigger(p) pulse count (0, 1) = empty ricky.DiskCount
5 = wait(s) seconds
6 = roller(0,r) degrees (1,v) velocity
*/
/*double test_route[][5] = {{0, 230, 230, 180, 0},
                          {1, 600, 600, 90, 0.5},
                          //{1, -200, 300, 270, 0.7},
                          {1, 230, 230, 180, 0.5}};
int test_route_length = sizeof(test_route) / sizeof(double) / 5;*/

double leftdiskandrollerup[][5] = {
    {0, -814, 255, 270, 0}, // set origin
    {1, -814, 220, 270, .7},
    {6, 0, 180, 0, 0},         // spin roller
    {3, 75, 0, 0, 0},          // spin up flywheel
    {1, -1850, 1220, 244, .7}, // move to middle of court
    {4, 2, 0, 0, 0},           // shoot two disks
    {3, 0, 0, 0, 0}            // spindown flywheel
};

const int leftdiskandrollerup_length = sizeof(leftdiskandrollerup) /
                                       sizeof(double) /
                                       5; // calculate length of array
double bothrollers[][5] = {
    {0, -814, 255, 270, 0},  // set origin
    {1, -814, 220, 270, .7}, // touch against roller
    {6, 0, 180, 0, 0},       // spin roller
    //{7, -1880, 1220, 244, 0.7}, // midpoint dont hit barrier
    //{7, -2700, 1700, 190, 0.7}, // avoid outside
    {7, -3090, 2500, 270, 0.7}, // goto next roller but stay inside lines
    {1, -3090, 2500, 180, 0.5}, // shpinmove
    {1, -3110, 270, 180, 0.7},  //
    {6, 0, 180, 0, 0}           // spin roller
};
const int bothrollers_length = sizeof(bothrollers) / sizeof(double) / 5;

double skills[][5] = {
    {0, -814, 255, 270, 0},    // initalize
    {1, -814, 220, 270, 0.7},  // touch against roller
    {6, 0, 360, 0, 0},         // spin 180 degrees to red
    {1, -220, 814, 10, 0.7},   // move to next roller
    {6, 0, 360, 0, 0},         // spin 180 degrees to red on second roller
    {3, 70, 0, 0, 0},          // start flywheel
    {1, -814, 1814, 275, 0.7}, // move to shoot disks
    {4, 2, 0, 0, 0},           // shoot two disks
    {}

};
const int skills_length = sizeof(skills) / sizeof(double) / 5;

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
      ricky.Destination = true;
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
      if (routine[i][2] == 1) {      // empty hopper
        TriggerPulse(Ricky.DiskCount);
      } else {
        TriggerPulse(int(routine[i][1]));
      }
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
    } else if (routine[i][0] == 7) { // waypoint
      ricky.TargetXAxis = routine[i][1];
      ricky.TargetYAxis = routine[i][2];
      ricky.TargetTheta = routine[i][3];
      ricky.TargetSpeed = routine[i][4];
      ricky.StartXAxis = ricky.CurrentXAxis;
      ricky.StartYAxis = ricky.CurrentYAxis;
      ricky.StartTheta = ricky.CurrentThetaValue;
      ricky.Destination = false;
      vex::task::sleep(250);
      while (
          ricky.EngineBusy &&
          !ricky.TravelImpeded) { // wait for the engine to finish or get stuck
        vex::task::sleep(50);
      }
    }
  }
  ricky.AutoDone = true;
}

int AutonomousRoutineDeamon() { // Main engine loop
  extern int test_route_length;
  AutonomousIndexer(bothrollers,
                    bothrollers_length); // runs through the given routine
  return 1;
};
