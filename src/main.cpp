/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Castillo                                                  */
/*    Created:      Thu Oct 12 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// L1                   motor         11              
// L2                   motor         20              
// R2                   motor         10              
// R1                   motor         1               
// Controller1          controller                    
// Mash                 motor         6               
// StartDirect          vision        12              
// ---- END VEXCODE CONFIGURED DEVICES ----



#include "vex.h"

using namespace vex;




#include "vex.h"

using namespace vex;



// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/




void leftpower(double percet) {
  // Left spin until stopped

  L1.spin(forward, percet, pct) ;
  L2.spin(forward, percet, pct) ;

}

void rightpower(double percet) {
  // Right spin until stopped

  R1.spin(forward, percet, pct) ;
  R2.spin(forward, percet, pct) ;

}

void fullpower(double leftpow, double rightpow) {
  // Full spin until stopped

  R1.spin(forward, rightpow, pct) ;
  R2.spin(forward, rightpow, pct) ;

  L1.spin(forward, leftpow, pct) ;
  L2.spin(forward, leftpow, pct) ;
}



int hgreen = 0; // Height of Green Object for distance measurement
int wgreen = 0; // Width of Green Object for distance measurement
int adjust_turn = 0; // Will turn a specified constant
double CenterFOV = 157; // Center of pixels
int CenterGreen = 0;
int Targetarea = 53172; // Area which robot should stop at
double Ki = 0;
double Kp = 0;
double Kd = 0;

int Realarea = 0;

double powe = 0;
double Bcons = -0.4623 ; 
double Acons = 2804;
double distance_temp = 0;
double distance_p = 0; //Distance conversion from area of pixels into distance in cm
double target_distance = 10;
double error_turn = 0;
double adjust_kp = 0;

void autoadjust() {
  StartDirect.takeSnapshot(StartDirect__GREENT);
  hgreen = StartDirect.objects[0].height;
  wgreen = StartDirect.objects[0].width;
  Realarea = hgreen * wgreen ; // Used to calculate our distance from the object

  CenterGreen = StartDirect.objects[0].centerX;

  powe = 100 ; // Full power which will be subtracted based on  the adjust
  // Brute force constant, this is the minimum the robot will adjust if it has to turn
  // Kp will change depending on how well it is working (It is also not really Kp just a constant)

  distance_temp = pow(Realarea, Bcons) ;
  distance_p = distance_temp*Acons ; // Distance in centimeters from area Realarea
  error_turn = 157 - CenterGreen ;
  
  

  // Alright so the main problem is I want to combine advancing and turning
  // For this I need to calculate the change in each motor
  // Luckily its more like calculating the change in 2 motors since sides can be grouped
  // First I will make a function that makes the more move by degrees
  // Then I can use a PID or something to calculate the movements
  // How do I make the PID? 
  // Let us think about this like having a target
  // We have to aim for the object to be in the center of our robot
  // We also have to aim for the object being as close as possible

  // Our targets will be 157 for the turn and 53172 for distance

  // So we will have speed x for both motors minus a correction which will adjust everything

  // Our distance target will serve as a stopper and will be used to help the robot steer
  // Think about it this way, if we are very close to the object and its to the side
  // We want to do a sharp turn
  // But if we are far away we could do a steering method by using a proportional calculation
  // The farther we are the less we have to turn 
  // The closer we are the more we have to turn 
  // The problem with this is that if we use an exponential turning function
  // We might not turn enough in the beginning and lose sight of our object
  // For this reason our steer always has to be constant (mostly)
  // This way it does not do a dangerous sharp turn at the end when it is near the object
  // Basically we will have an already set turn which is the adjust's minimum
  // That way we can always turn before we reach it
  // We will call this minimum kp



  // Also if we stop seeing the object we will just stop and then I will programm a back up plan
  // I do not know what will be the back up yet though, I am thinking of using the black bars

  if (StartDirect.objects[0].exists) {
    if (distance_p > target_distance) {
      // Alright now it will move until it will repeat the code until it hits the object
      // Unfortunately, it does not really have a sense of direction yet
      fullpower(powe - Ki, powe - Ki); //Ki is temporary just to have something there


    }
  }

}



void forwardstime(double times) {
  L1.spin(forward,50,pct);
  L2.spin(forward,50,pct);
  R1.spin(forward,50,pct);
  R2.spin(forward,50,pct);

  wait(times, msec);

  L1.stop();
  L2.stop();
  R1.stop();
  R2.stop();
}

void backwardstime(double times) {
  L1.spin(reverse,50,pct);
  L2.spin(reverse,50,pct);
  R1.spin(reverse,50,pct);
  R2.spin(reverse,50,pct);

  wait(times, msec);

  L1.stop();
  L2.stop();
  R1.stop();
  R2.stop();
}

void leftime(double times) {
  L1.spin(reverse,50,pct);
  L2.spin(reverse,50,pct);
  R1.spin(forward,50,pct);
  R2.spin(forward,50,pct);

  wait(times, msec);

  L1.stop();
  L2.stop();
  R1.stop();
  R2.stop();
}

void rightime(double times) {
  L1.spin(forward,50,pct);
  L2.spin(forward,50,pct);
  R1.spin(reverse,50,pct);
  R2.spin(reverse,50,pct);

  wait(times, msec);

  L1.stop();
  L2.stop();
  R1.stop();
  R2.stop();
}

// Hey Joao all of these functions operate on time sooooo
// It would be great if you could make a function that operates on degrees
// I haven't had a lot of time to do it since I have been working on vision sensor
// Also these autonomous is pretty useless it just goes forward and backwards

void autonomous(void) {

  L1.stop();
  L2.stop();
  R1.stop();
  R2.stop();

  forwardstime(1000);

  backwardstime(300);

  forwardstime(500);

  backwardstime(200);




}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  vexcodeInit();

  double pos = Mash.position(degrees);
  //pos2 will be set degrees before launching

  while (1) {
    float ch3=-1*Controller1.Axis3.value();
    float ch2=Controller1.Axis2.value();
    


    //If moving forward is more than 10 then advance if less then do nothing,
    //I do not understand the axis of the robot very well, have to investigate better
    if(fabs(ch3)<10)
    {
      ch3=0;
    }
    if(fabs(ch2)<10)
    {
      ch2=0;
    }

    L1.spin(reverse,ch3,pct);
    L2.spin(reverse,ch3,pct);
    R1.spin(forward,ch2,pct);
    R2.spin(forward,ch2,pct);

    // ch2 and ch2 are percentage/power defined by analog stick

    if (Controller1.ButtonL1.pressing()) {
      Mash.spin(forward,40,pct);
    } else if (Controller1.ButtonL2.pressing()) {
      Mash.spin(reverse,40,pct);
    } else {
      Mash.setVelocity(40,percent);
      Mash.spinToPosition(pos,degrees);
    }
    
    // sudo code variable x degrees position, label current position, 
    // press button up arrow set degrees for launch mode


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
