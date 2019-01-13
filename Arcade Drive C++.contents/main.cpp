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

/**SPINNER AND REVERSAL VARIABLES END*/

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

motor lmotors[] {Motor11dl, Motor01dl, Motor03dl};
motor rmotors[] {Motor04dr, Motor16dr, Motor02dr};
const int NUM_MOTORS = 3; // per side

/* Scaling constants */
const float DEADZONE = 0.10;
const float MAX_MOTOR_VAL = 100.0;
const float MAX_JOY_VAL = 100.0;

/**
 * Scales a joystick input value (-1 to 1) to a float 
 * between -1 and 1, representing percent motor output.
 * (Negative being reversed). Quadratically scales.
 */
int scaleJoystick(int input) {
    float result = 0;
    input = (input < 0 ? -input : input);
    
    /* Input exceeds threshold to function */
    if (input > DEADZONE) {
        /* Determine a ratio of the input to the x value range 
         * that joystick inputs can have. This ignores the DEADZONE,
         * for example, if my input was 20, graphed on an x-axis, it
         * would look like
         *                         |---------B--------|
         *                     |-C-|-A-|
         * -100 -------------- 0 ---- 20 ------------ 100
         * 
         * The expression below calculates the ratio of A to B, where
         * C is the deadzone (not included). */
        result = (input - DEADZONE) / (MAX_JOY_VAL - DEADZONE);

        /* Quadratically scale the result, making lower input 
         * values grow more slowly than higher ones. At max joystick 
         * input (100) result should be equal to 100, giving max motor
         * strength */
        result *= result;

        /* If input was initially negative make output negative */
        if (input < 0)
            result *= -1;
    }
    return (int)result;
}

void arcadedrive(bool reversed) {
    int px = scaleJoystick(Controller1.Axis1.position(percentUnits::pct));
    int py = scaleJoystick(Controller1.Axis2.position(percentUnits::pct));
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
void Spinner() {
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
        Spinner();
        // Drive code
        arcadedrive(reversed);
    }
}
