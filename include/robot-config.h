using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern motor L1;
extern motor L2;
extern motor R2;
extern motor R1;
extern controller Controller1;
extern motor Mash;
extern signature StartDirect__GREENT;
extern signature StartDirect__REDT;
extern signature StartDirect__BLUET;
extern signature StartDirect__BAR;
extern signature StartDirect__SIG_5;
extern signature StartDirect__SIG_6;
extern signature StartDirect__SIG_7;
extern vision StartDirect;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );