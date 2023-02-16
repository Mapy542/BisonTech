using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor FL;
extern motor RL;
extern motor FR;
extern motor RR;
extern inertial Gyroscope;
extern encoder y;
extern encoder x;
extern motor Flywheel1;
extern motor Flywheel2;
extern motor Intake;
extern digital_out Trigger;
extern digital_out EndgameLaunch;
extern motor Roller;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );