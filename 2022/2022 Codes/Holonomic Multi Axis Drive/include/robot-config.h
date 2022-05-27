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
extern motor LiftLeft;
extern motor LiftRight;
extern motor BaseLock;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );