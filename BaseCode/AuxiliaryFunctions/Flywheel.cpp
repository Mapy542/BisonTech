#include "../VectorEngine/Robot_Telemetry_Structure.cpp"
#include "vex.h"

void ManualFlywheel() {
  extern Robot_Telemetry ricky;
  int power =
      (fabs(1828 - ricky.CurrentYAxis) + fabs(1828 - ricky.CurrentXAxis)) /
          1828 * 100 +
      40;

  if (Controller1.ButtonB.pressing()) {
    Flywheel1.setVelocity(power, percent);
    Flywheel2.setVelocity(power, percent);
  }

  if (Controller1.ButtonA.pressing()) {
    Flywheel1.setVelocity(0, percent);
    Flywheel2.setVelocity(0, percent);
  }
  Flywheel1.spin(forward);
  Flywheel2.spin(forward);

  if(Controller1.ButtonX.pressing()){
    while(Controller1.ButtonX.pressing()){
      Flywheel1.spin(reverse);
      Flywheel2.spin(reverse);

      Flywheel1.setVelocity(10, percent);
      Flywheel1.setVelocity(10, percent);
    }
    else{
      Flywheel1.setVelocity(0, percent);
      Flywheel2.setVelocity(0, percent);
        
      Flywheel1.spin(forward);
      Flywheel2.spin(forward);
    }
  }

  if (Controller1.ButtonL1.pressing()) {
    Trigger.set(true);
  } else {
    Trigger.set(false);
  }
<<<<<<< Updated upstream

=======
}

void FlywheelVelocity(int velocity) {
  Flywheel1.setVelocity(velocity, percent);
  Flywheel2.setVelocity(velocity, percent);
  Flywheel1.spin(forward);
  Flywheel2.spin(forward);
}

void TriggerPulse(int pulses) {
  for (int i = 0; i < pulses; i++) {
    Trigger.set(true);
    vex::task::sleep(500);
    Trigger.set(false);
    vex::task::sleep(2000);
  }
>>>>>>> Stashed changes
}