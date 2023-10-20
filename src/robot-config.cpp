#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor L1 = motor(PORT11, ratio6_1, true);
motor L2 = motor(PORT20, ratio6_1, true);
motor R2 = motor(PORT10, ratio6_1, false);
motor R1 = motor(PORT1, ratio6_1, false);
controller Controller1 = controller(primary);
motor Mash = motor(PORT6, ratio6_1, false);
/*vex-vision-config:begin*/
signature StartDirect__GREENT = signature (1, -5513, -3941, -4728, -4637, -2519, -3578, 2.5, 0);
signature StartDirect__REDT = signature (2, 5405, 8301, 6854, -323, 275, -24, 2.5, 0);
signature StartDirect__BLUET = signature (3, -3769, -2783, -3276, 8551, 12031, 10290, 2.5, 0);
signature StartDirect__BAR = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vision StartDirect = vision (PORT12, 50, StartDirect__GREENT, StartDirect__REDT, StartDirect__BLUET, StartDirect__BAR);
/*vex-vision-config:end*/

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1RightShoulderControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // check the ButtonR1/ButtonR2 status to control Mash
      if (Controller1.ButtonR1.pressing()) {
        Mash.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Mash.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Mash.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}