#include "vex.h"

int VisionDeamon() {
  extern signature VISION__DISK;
  while (true) {
    Vision.takeSnapshot(VISION__DISK);
    vex::task::sleep(50);
  }
}