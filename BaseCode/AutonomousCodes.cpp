#include "AuxiliaryFunctions\BackBase.cpp"
#include "AuxiliaryFunctions\BaseLock.cpp"
#include "AuxiliaryFunctions\CascadeLift.cpp"
#include "AuxiliaryFunctions\PneumaticPush.cpp"
#include "KinematicEngine\Engine.cpp"

//#include "VectorEngine\Engine.cpp"
#include "vex.h"

// JENA CODE HERE

// BEN CODE HERE

void ThrustTest() {
  vex::task::sleep(25);
  while (true) {
    Brain.Screen.print("%f", Brain.Battery.current());
    vex::task::sleep(25);
  }
};
/*
void SkillsRun() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180.0, degrees);
  // pickup red
  SendLiftto(-300);
  RodKnock();
  Destination(3260.0, 550.0, 180.0, 0.6);
  LiftLeft.stop();
  LiftRight.stop();
  Lock_Base();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  SendLiftto(300);
  // drop red
  Waypoint(3260.0, 2000.0, 180.0, 0.7);
  Destination(2700.0, 2400.0, 90.0, 0.6);
  SendLiftto(-300.0);
  Unlock_Base();
  Destination(3220.0, 2400.0, 90.0, 0.7);
  // pickup blue
  Waypoint(3250.0, 3300.0, 90.0, 0.5);
  Destination(2935.0, 3300.0, 90.0, 0.5);
  Lock_Base();
  SendLiftto(600);
  Waypoint(3250.0, 3220.0, 90.0, 0.5);
  Waypoint(2820.0, 2400.0, 90.0, 0.7);
  // bulldoze yellow 1
  Waypoint(2820.0, 900.0, 90.0, 0.7);
  Destination(2820.0, 1100.0, 90.0, 0.7);
  // spin move blue
  Unlock_Base();
  SendLiftto(-600.0);
  Waypoint(2500.0, 1100.0, 300.0, 0.7);
  Waypoint(1800.0, 1100.0, 270.0, 0.7);
  // bulldoze yellow 2
  Waypoint(1780.0, 1300.0, 270.0, 0.7);
  Waypoint(1780.0, 2300.0, 270.0, 0.7);
  Waypoint(1300.0, 2300.0, 270.0, 0.7);
  // pickup blue
  Waypoint(1200.0, 2775.0, 90.0, 0.7);
  Destination(600.0, 2775.0, 90.0, 0.5);
  Lock_Base();
  SendLiftto(300.0);
  // yellow number 3
  Waypoint(1000.0, 2300.0, 250.0, 0.7);
  Waypoint(1200.0, 1300.0, 250.0, 0.7);
  // drop blue 2
  Destination(1400.0, 1350.0, 300.0, 0.5);
  Unlock_Base();
  SendLiftto(-300.0);
  Waypoint(1130.0, 1350.0, 270.0, 0.6);
  Waypoint(350.0, 1350.0, 270.0, 0.5);
  // pick red
  Waypoint(350.0, 450.0, 270.0, 0.5);
  Destination(650.0, 450.0, 270.0, 0.5);
  Lock_Base();
  SendLiftto(600.0);
  Waypoint(350.0, 450.0, 270.0, 0.7);
  // yeet
  Destination(430.0, 2600.0, 270.0, 0.7);
  Unlock_Base();
  SendLiftto(-600.0);
};

// checked worlds
void RightYellowLast() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180.0, degrees);
  // pickup red
  SendLiftto(-300.0);
  Destination(3260.0, 550.0, 180.0, 0.6);
  LiftLeft.stop();
  LiftRight.stop();
  DynamicBoundsDetection();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  SendLiftto(300);
  // drop red
  Destination(2800.0, 500.0, 270.0, 0.6);
  Unlock_Base();
  SendLiftto(-300.0);
  Waypoint(2600.0, 500.0, 270.0, 0.7);
  Waypoint(2730.0, 1250.0, 180.0, 0.7);
  Destination(2730.0, 1600.0, 180.0, 0.6);
  Lock_Base();
  SendLiftto(150);
  Destination(2730.0, 300.0, 180.0, 0.5);
  Unlock_Base();
  SendLiftto(-150);
  LiftLeft.stop();
  LiftRight.stop();
};

// checked worlds
void RightYellowFirst() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180.0, degrees);
  // avoid home base
  // Waypoint(3000.0, 350.0, 180.0, 0.7);
  XArc(2700.0, 1000.0, 180.0, 0.7, true);
  SendLiftto(-300);
  RodKnock();
  // pickup yellow
  Destination(2710.0, 1650.0, 180.0, 0.6);
  LiftLeft.stop();
  LiftRight.stop();
  DynamicBoundsDetection();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  SendLiftto(300);
  // push red
  YArc(3150, 1400, 180, .7, false);
  Destination(3200.0, 600, 180, 0.6);
  SendLiftto(-300);
  Unlock_Base();
  LiftLeft.stop();
  LiftRight.stop();
};

// checked worlds
void LeftYellowFirst() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180, degrees);
  Destination(830, 1000, 180, .7);
  SendLiftto(-300);
  RodKnock();
  Destination(830, 1700, 180, 0.6);
  LiftLeft.stop();
  LiftRight.stop();
  DynamicBoundsDetection();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  Destination(1500, 900, 270, .7);
  Unlock_Base();
  Waypoint(300, 900, 270, .7);
  Waypoint(300, 300, 270, .7);
  Destination(750, 300, 270, .7);
  Lock_Base();
  SendLiftto(650);
  Waypoint(600, 600, 270, .7);
  Destination(900, 500, 1, .5);
  Unlock_Base();
  SendLiftto(-200);
  Destination(900, 700, 1, .7);
};

// checked worlds
void RightSideMiddle() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180.0, degrees);
  // send in front of middle
  XArc(2450.0, 1400.0, 130, 0.7, true);
  SendLiftto(-300);
  RodKnock();
  // pickup middle yellow
  Destination(2000.0, 1900.0, 130, 0.6);
  LiftLeft.stop();
  LiftRight.stop();
  DynamicBoundsDetection();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  SendLiftto(300);
  // back to home zone
  Destination(2900.0, 400.0, 130.0, 0.7);
  SendLiftto(-300);
  LiftLeft.stop();
  LiftRight.stop();
};

// checked worlds
void LeftSideMiddle() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180.0, degrees);
  // send in front of middle
  Destination(800.0, 1200.0, 180, 0.7);
  SendLiftto(-300);
  Destination(1100.0, 1200.0, 225, 0.7);
  RodKnock();
  // pickup middle yellow
  Destination(1700.0, 1700.0, 225, 0.6);
  LiftLeft.stop();
  LiftRight.stop();
  DynamicBoundsDetection();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  SendLiftto(300);
  // back to home zone
  Destination(700.0, 700.0, 225, 0.6);
  SendLiftto(-300);
  Unlock_Base();
  LiftLeft.stop();
  LiftRight.stop();
};

// expirmental
void RightYellowBack() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180.0, degrees);
  // avoid home base
  Waypoint(3000.0, 350.0, 180.0, 0.7);
  Destination(2700.0, 1000.0, 1.0, 0.7);
  Extend_Back();
  // pickup yellow
  Destination(2700.0, 1750.0, 1.0, 0.5);
  wait(.5, seconds);
  Destination(2700, 1750, 0, .7);
  wait(.5, seconds);
  CloseBack();
  Retract_Back();
  SendLiftto(-300);
  LiftLeft.stop();
  LiftRight.stop();
  RodKnock();
  // push red
  Waypoint(3230, 1500, 12, .4);
  Destination(3230.0, 600, 12, 0.4);
  Lock_Base();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  LiftLeft.stop();
  LiftRight.stop();
};

// expirmental
void SkillsV2() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180.0, degrees);
  // pickup blue
  SendLiftto(-300);
  RodKnock();
  Destination(3260.0, 550.0, 180.0, 0.6);
  LiftLeft.stop();
  LiftRight.stop();
  DynamicBoundsDetection();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  SendLiftto(400);
  // Drive to blue platform
  Waypoint(3260.0, 2600.0, 200.0, 0.7);
  Destination(2900.0, 3100.0, 180.0, 0.6);
  SendLiftto(700);
  // level platform
  Destination(1500, 3100.0, 180.0, 0.6);
  Destination(1800, 3100, 180, 0.7);
  // drop first blue
  SendLiftto(-30.0);
  Unlock_Base();
  Destination(1720, 2700.0, 180.0, 0.7);
  SendLiftto(-1070);

  // BACK HALF HYBRID
  // pickup right yellow
  Waypoint(1720, 2800, 350, .7);
  Waypoint(2633.75, 2800, 350, .7);
  Destination(2633.75, 2133, 350, .7);
  Lock_Base();
  SendLiftto(1100);
  // drop right yellow
  Destination(2750, 3100, 180, .7);
  Destination(2000, 3100, 180, .7);
  SendLiftto(-30);
  Unlock_Base();
  Waypoint(1900, 2550, 200, .7);
  Destination(2000, 2700, 180, .7);
  SendLiftto(-1070);
  // pickup left yellow
  Destination(1950, 2800, 12, .7);
  Destination(940, 2800, 13, .7);
  Destination(940, 2133, 10, .5);
  Lock_Base();
  SendLiftto(1100);
  // drop left yellow
  Destination(890, 3100, 180, .7);
  Destination(1650, 3100, 180, .7);
  // SendLiftto(-30);
  Unlock_Base();
  Destination(1650, 2700, 180, .7);
  SendLiftto(-1100);
  // bulldoze middle yellow
  Waypoint(1700, 2600, 180, .7);
  Destination(1700, 900, 180, .7);
}

void ShowOffArc() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180.0, degrees);
  while (true) {
    XArc(700, 600, 180, .5, false);
    XArc(0, 0, 180, .5, false);
    YArc(-700, -600, 180, .5, false);
    YArc(0, 0, 180, .5, true);
  }
}

// Attemp to shove base further onto platform with ram feature.
void SkillsV2Ramming() {
  while (Gyroscope.isCalibrating()) {
    task::sleep(50);
  }
  Gyroscope.setHeading(180.0, degrees);
  // pickup blue
  SendLiftto(-300);
  RodKnock();
  Destination(3260.0, 550.0, 180.0, 0.6);
  LiftLeft.stop();
  LiftRight.stop();
  DynamicBoundsDetection();
  wait(0.15, seconds);
  Unlock_Base();
  wait(0.15, seconds);
  Lock_Base();
  SendLiftto(400);
  // Drive to blue platform
  Waypoint(3260.0, 2600.0, 200.0, 0.7);
  Destination(2900.0, 3100.0, 180.0, 0.6);
  SendLiftto(700);
  // level platform
  Destination(1500, 3100.0, 180.0, 0.6);
  Destination(1800, 3100, 180, 0.7);
  // drop first blue
  SendLiftto(-30.0);
  Unlock_Base();
  Ram(1800, 3150, 180);
  Destination(1720, 2700.0, 180.0, 0.7);
  SendLiftto(-1070);

  // BACK HALF HYBRID
  // pickup right yellow
  Waypoint(1720, 2800, 350, .7);
  Waypoint(2633.75, 2800, 350, .7);
  Destination(2633.75, 2133, 350, .7);
  Lock_Base();
  SendLiftto(1100);
  // drop right yellow
  Destination(2750, 3100, 180, .7);
  Destination(2000, 3100, 180, .7);
  SendLiftto(-30);
  Unlock_Base();
  Ram(2000, 3150, 180);
  Waypoint(1900, 2550, 200, .7);
  Destination(2000, 2700, 180, .7);
  SendLiftto(-1070);
  // pickup left yellow
  Destination(1950, 2800, 12, .7);
  Destination(940, 2800, 13, .7);
  Destination(940, 2133, 10, .5);
  Lock_Base();
  SendLiftto(1100);
  // drop left yellow
  Destination(890, 3120, 180, .7);
  Destination(1650, 3120, 180, .7);
  // SendLiftto(-30);
  Unlock_Base();
  Ram(1650, 3150, 180);
  Destination(1650, 2700, 180, .7);
  SendLiftto(-1100);
  // bulldoze middle yellow
  Waypoint(1700, 2600, 180, .7);
  Destination(1700, 900, 180, .7);
}*/