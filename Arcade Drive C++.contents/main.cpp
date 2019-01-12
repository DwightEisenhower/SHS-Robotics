#include "robot-config.h"

/* Define additional digital outputs.
 * Follow this format:
 * 
 * vex::digital_out <var name> = vex::digital_out( PORT );
 * 
 * Where PORT is a Brain.ThreeWirePort.[letter] = https://www.robotmesh.com/docs/vexv5-cpp/html/classvex_1_1triport.html
 * Ie Brain.ThreeWirePort.A
 */

/**SPINNER AND REVERSAL CODE START*/
/* Define global variables because why not */
bool spinnerOn = false;

/* Messages */
const char* reverseMessageOn = "Motors are REVERSED";
const char* reverseMessageOff = "Motors are NORMAL  ";
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

/**SPINNER AND REVERSAL VARIABLES END*/


motor lmotors[] {Motor11dl, Motor01dl, Motor03dl};
motor rmotors[] {Motor04dr, Motor16dr, Motor02dr};
const int NUM_MOTORS = 3; // per side

void arcadedrive(bool reversed) {
    int px = Controller1.Axis1.position(percentUnits::pct);
    int py = Controller1.Axis2.position(percentUnits::pct);
    if (reversed) {
        py *= -1;
    }
    int lp = py + px;
    int rp = py - px;

    Controller1.Screen.setCursor(2, 0);
    Controller1.Screen.print("Lrpm = %4d%%", lp);
    Controller1.Screen.setCursor(3, 0);
    Controller1.Screen.print("Rrpm = %4d%%", rp);

    for (int i = 0; i < NUM_MOTORS; i++) {
        lmotors[i].spin(vex::directionType::fwd, lp, percentUnits::pct);
        rmotors[i].spin(vex::directionType::fwd, rp, percentUnits::pct);
    }
}

/**
 * Deal with the spinner thing at the front 
 * of the robot. Pls refactor.
 */
void spinner() {
    if (Controller1.ButtonX.pressing()) {
        spinnerOn = !spinnerOn;
    }
    if (spinnerOn) {
        Motor05sp.spin(directionType::fwd,600,velocityUnits::rpm);
    } else {
        Motor05sp.stop();
    }
}

int main() {
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
            Controller1.Screen.setCursor(1, 0)   ;
            Controller1.Screen.print(reversed ? reverseMessageOn : reverseMessageOff);
            Controller1.rumble(rumbleCode);
        }
        // Spinny thing code
        spinner();
        // Drive code
        arcadedrive(reversed);
    }
}