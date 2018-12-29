#include "robot-config.h"

/* Remember to backup code before closing editor
 * Lost 4 hours of work 
 */

/* Define global variables because why not */
vex::competition    Competition;
bool spinnerOn = false;
int autonState = 1;

/* Messages */
const char* reverseMessageOn = "Motors are REVERSED";
const char* reverseMessageOff = "Motors are NORMAL";
const char* rumbleCode = ".-";

/**
 * Reversing of motors. If reversed is true then
 * directions.fwd and directions.rev will be the opposite of
 * what they're intended to do (Ie directions.fwd = vex::directionType::rev)
 *
 * Upon changing the state of reversed, the reversedTimeout will be incremented
 * to REVERSE_TIMEOUT and decrease to 0 each iteration of the loop.
 */
bool reversed = false; // Are all motors reversed?
const int MAX_REVERSE_TIMEOUT = 7; // Max iterations before state can be changed
int reversedTimeout = 0; // Min timeout between motor reversals

/**
 * Stores all direction types. Use this instead of 
 * vex::directionType if you want the motor to be 
 * reversed by the reversed variable 
 *
 * Ie. Do directions.fwd instead of vex::directionType::fwd
 */
struct directionsStruct {
    vex::directionType fwd;
    vex::directionType rev;
} directions;


/**
 * Deal with the spinner thing at the front 
 * of the robot. Pls refactor.
 */
void doSpinner() {
    if (Controller1.ButtonY.pressing()) {
        spinnerOn = !spinnerOn;
    }
    if (spinnerOn) {
        Motor07.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } else {
        Motor07.stop();
    }
}

void usercontrol(void) {
    while(true) {
        /**
         * Checking of motor reversal code. If the B button
         * is pressed all motors will be toggled to reverse
         * direction
         */
        reversedTimeout = reversedTimeout < 0 ? 0 : reversedTimeout - 1;
        if(Controller1.ButtonB.pressing() && reversedTimeout <= 0) {
            reversed = !reversed;
            reversedTimeout = MAX_REVERSE_TIMEOUT;
            
            /* Display the current reversal state */
            Controller1.Screen.clearScreen();
            Controller1.Screen.print(reversed ? reverseMessageOn : reverseMessageOff);
            Controller1.rumble(rumbleCode);
        }
        
        directions.fwd = !reversed ? vex::directionType::fwd : vex::directionType::rev;
        directions.rev = !reversed ? vex::directionType::rev : vex::directionType::fwd;

        // Spinny thing code
        doSpinner();

        // Right motor, vertical axis of left joystick
        Motor01.spin(directions.fwd, Controller1.Axis2.position(vex::percentUnits::pct), vex::velocityUnits::pct);
        Motor02.spin(directions.fwd, Controller1.Axis2.position(vex::percentUnits::pct), vex::velocityUnits::pct);
        Motor05.spin(directions.fwd, Controller1.Axis2.position(vex::percentUnits::pct), vex::velocityUnits::pct);
        
        // Left motor, vertical axis of right joystick
        Motor03.spin(directions.fwd, Controller1.Axis3.position(vex::percentUnits::pct), vex::velocityUnits::pct);
        Motor04.spin(directions.fwd, Controller1.Axis3.position(vex::percentUnits::pct), vex::velocityUnits::pct);
        Motor16.spin(directions.fwd, Controller1.Axis2.position(vex::percentUnits::pct), vex::velocityUnits::pct);
        
        task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources. 
    }
}

/**
 * Stop all wheel motors 
 */
void stopAllMotors() {
    Motor01.stop();
    Motor02.stop();
    Motor03.stop();
    Motor04.stop();
    Motor05.stop();
    Motor16.stop();
}

/**
 * moveStraight with power, forward direction and time
 *
 * @param power=100  Power
 * @param fwd=true   Go forward?
 * @param time=1000  Delay in ms before stopping
 */
void moveStraight (int power=100, bool fwd=true, int time=1000) {
    vex::directionType dir = fwd ? vex::directionType::fwd : vex::directionType::rev;
    
    Motor01.spin(dir, power, vex::velocityUnits::pct);
    Motor02.spin(dir, power, vex::velocityUnits::pct);
    Motor05.spin(dir, power, vex::velocityUnits::pct);
    Motor03.spin(dir, power, vex::velocityUnits::pct);
    Motor04.spin(dir, power, vex::velocityUnits::pct);
    Motor16.spin(dir, power, vex::velocityUnits::pct);
    
    task::sleep(time);
    stopAllMotors();
}

/**
 * Move forward a certain distance 
 * @param distance  Distance to move in meters
 * @param fwd=true  Move forward or backwards?
 */
void moveStraightDistance(double distance, bool fwd=true) {
    // Robot moves at approximately 1.3 m/s
    moveStraight(100, fwd, distance / 1.3 * 1000);
}

/*
 * Attempt to rotate an angle. Negative is left, 
 * positive is right. Angle is in degrees. Not 100% accurate.
 *
 * @param angle Angle to rotate in degrees
 */
void rotate(int angle) {
    /* If turning left, right is forward, left backwards
     * If turning right, right is backwards, left forwards */
    vex::directionType dirL = angle < 0 ? vex::directionType::fwd : vex::directionType::rev;
    vex::directionType dirR = angle > 0 ? vex::directionType::fwd : vex::directionType::rev;

    /* Right side motor */
    Motor01.spin(dirR, 100, vex::velocityUnits::pct);
    Motor16.spin(dirR, 100, vex::velocityUnits::pct);
    Motor02.spin(dirR, 100, vex::velocityUnits::pct);

    /* Left side motor */
    Motor05.spin(dirL, 100, vex::velocityUnits::pct);
    Motor03.spin(dirL, 100, vex::velocityUnits::pct);
    Motor04.spin(dirL, 100, vex::velocityUnits::pct);
    
    /* Calculate time to rotate (Rotates at around 0.36 deg/ms) */
    int absAng = angle < 0 ? -angle : angle;
    int time = absAng / 9 * 25;
    
    /* Slight adjustments for lower angles */
    if (absAng < 80) time += 1500 / absAng;
    else if (absAng < 190) time += 2300 / absAng;
    else time -= 700 / absAng;
    
    task::sleep(time);
    stopAllMotors();
}

/**
 * Auton: go backwards to hit the flag, then forward
 * and go to platform 
 */
void auton12(bool isRed) {
    moveStraightDistance(1.78, false);
    task::sleep(300);
    moveStraightDistance(1.78);
    task::sleep(300);
    rotate(isRed ? 120 : -120);
    moveStraightDistance(1.58);
}

/**
 * Auton: Rotate 35 degrees then go forward onto
 * the platform 
 */
void auton34(bool isRed) {
    rotate(isRed ? -35 : 35);
    moveStraightDistance(1.58);
}

/**
 * Run auton
 */
void auton() {
    switch (autonState) {
        case 1: auton12(true);
                break;
        case 2: auton12(false);
                break;
        case 3: auton34(true);
                break;
        case 4: auton34(false);
                break;
    }
}

/**
 * Display the current auton state
 */
void displayCurrentAutonState() {
    switch (autonState) {
        case 1:
            Brain.Screen.setCursor(1,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("A1 - Red flag");
            
            Brain.Screen.setCursor(2,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("Face robot spinner away from flag");
            break;
        case 2:
            Brain.Screen.setCursor(1,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("A2 - Blue flag");
            
            Brain.Screen.setCursor(2,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("Face robot spinner away from flag");
            break;
        case 3:
            Brain.Screen.setCursor(1,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("A3 - Red platform");
            
            Brain.Screen.setCursor(2,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("Face robot spinner towards opposite side");
            break;
        case 4:
            Brain.Screen.setCursor(1,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("A4 - Blue platform");
            
            Brain.Screen.setCursor(2,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("Face robot spinner towards opposite side");
            break;
    }
    Brain.Screen.render();
}

/**
 * Runs when screen is pressed. Toggles the
 * auton state */
void screenpressed(void) {
    autonState = autonState % 4 + 1;
    displayCurrentAutonState();
    
    Brain.Screen.setCursor(3,0);
    Brain.Screen.clearLine();
    Brain.Screen.print("Press screen to toggle auton state");
}

/**
 * main
 */
int main() {
    Brain.Screen.render(true, false); // Enable double buffering for smoother drawing
    Brain.Screen.pressed(screenpressed);
    
    /* Display initial text */
    displayCurrentAutonState();
    Brain.Screen.setCursor(3,0);
    Brain.Screen.clearLine();
    Brain.Screen.print("Press screen to toggle auton state");
    
    // Competition.autonomous(auton);
    // Competition.drivercontrol(usercontrol);
    
    // auton();
    
    // Prevent main from exiting with an infinite loop.
    while(1) task::sleep(100); // Sleep the task for a short amount of time to prevent wasted resources.
}





// Solenoid test code
/* if (Controller1.ButtonA.pressing()) {
            Solenoid1Open.set(true);
        } else if (Controller1.ButtonB.pressing()) {
            Solenoid1Close.set(true);
        } */
        
/* Define additional digital outputs.
 * Follow this format:
 * 
 * vex::digital_out <var name> = vex::digital_out( PORT );
 * 
 * Where PORT is a Brain.ThreeWirePort.[letter] = https://www.robotmesh.com/docs/vexv5-cpp/html/classvex_1_1triport.html
 * Ie Brain.ThreeWirePort.A
 */
// vex::digital_out Solenoid1Open  = vex::digital_out(Brain.ThreeWirePort.A);
// vex::digital_out Solenoid1Close = vex::digital_out(Brain.ThreeWirePort.B);


