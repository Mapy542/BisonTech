#include "vex.h"

int VisionDeamon() {
  extern signature Vision__DISK;
  extern Robot_Telemetry ricky;
  while (true) {
    int DetectedObjects = Vision.takeSnapshot(Vision__DISK);
    if (DetectedObjects != 0) {
      bool OneDisk = false;
      for (int i = 0; i < DetectedObjects; i++) {
        if (Vision.objects[i].exists && Vision.objects[i].centerY > 160 &&
            !ricky.DiskInIntake) {
          ricky.DiskInIntake = true;
          ricky.DiskCount++;
          vex::task::sleep(200);
          OneDisk = true;
        } else if (Vision.objects[i].exists &&
                   !(Vision.objects[i].centerY > 160) && ricky.DiskInIntake &&
                   !OneDisk) {
          ricky.DiskInIntake = false;
        }
      }
    }

    if (ricky.DiskCount >= 3 &&
        Intake.velocity(percent) > 0) { // intake intelisense
      Intake.setVelocity(0, percent);   // stop intake if 3 disks are in the
                                      // hopper
      Intake.spin(forward);
    }
    vex::task::sleep(50);
  }
}