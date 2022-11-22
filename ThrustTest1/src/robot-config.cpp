#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FL = motor(PORT19, ratio18_1, false);
motor RL = motor(PORT20, ratio18_1, false);
motor FR = motor(PORT9, ratio18_1, true);
motor RR = motor(PORT10, ratio18_1, true);
inertial Gyroscope = inertial(PORT18);
encoder y = encoder(Brain.ThreeWirePort.A);
encoder x = encoder(Brain.ThreeWirePort.C);
motor LiftLeft = motor(PORT7, ratio36_1, false);
motor LiftRight = motor(PORT6, ratio36_1, true);
motor BaseLock = motor(PORT17, ratio18_1, true);
digital_out DigitalOutH = digital_out(Brain.ThreeWirePort.H);
motor BackMotor = motor(PORT16, ratio36_1, false);
digital_out DigitalOutG = digital_out(Brain.ThreeWirePort.G);
/*vex-vision-config:begin*/
vision Vision = vision (PORT1, 50);
/*vex-vision-config:end*/

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