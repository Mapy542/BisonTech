using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
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
extern signature Vision__DISK;
extern signature Vision__SIG_2;
extern signature Vision__SIG_3;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_MA_BALLS;
extern vision Vision;
extern motor Roller;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );