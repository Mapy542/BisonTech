#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FL = motor(PORT1, ratio6_1, false);
motor RL = motor(PORT6, ratio6_1, false);
motor FR = motor(PORT9, ratio6_1, true);
motor RR = motor(PORT10, ratio6_1, true);
inertial Gyroscope = inertial(PORT4);
encoder y = encoder(Brain.ThreeWirePort.C);
encoder x = encoder(Brain.ThreeWirePort.A);
motor Flywheel1 = motor(PORT2, ratio18_1, false);
motor Flywheel2 = motor(PORT5, ratio18_1, false);
motor Intake = motor(PORT14, ratio18_1, false);
digital_out Trigger = digital_out(Brain.ThreeWirePort.H);
digital_out EndgameLaunch = digital_out(Brain.ThreeWirePort.G);
motor Roller = motor(PORT19, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}