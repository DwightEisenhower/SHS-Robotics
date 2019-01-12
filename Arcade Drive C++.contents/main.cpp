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
 */
bool reversed = false; // Are all motors reversed?

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

/* Joystick rescaling - quadratic outside the dead zone */
const int DEADZONE = 10;
const double MAX_JOY_VAL = 127.0;  // need this double for division below

double scale_joystick(double input)
{
	double result = 0.;
	if (input > DEADZONE)
	{
		result = (input - DEADZONE) / (MAX_JOY_VAL - DEADZONE);
		result *= result;  // fine control, optional
	}
	else if (input < -DEADZONE)
	{
		result = (input + DEADZONE) / (MAX_JOY_VAL - DEADZONE);
		result *= -result;  // fine control, optional
	}
    else 
    {
        return 0;
    }
	return result / input;
}


motor lmotors[] {Motor11dl, Motor01dl, Motor03dl};
motor rmotors[] {Motor04dr, Motor16dr, Motor02dr};
const int NUM_MOTORS = 3; // per side

void arcadedrive() {
    double px = Controller1.Axis1.value();  //Gets the value of the joystick axis on a scale from -127 to 127.
    double py = Controller1.Axis2.value();

    // Print joystick values for information
    Controller1.Screen.setCursor(2, 0);
    Controller1.Screen.print("Xjoy = %5.1f ", px);
    Controller1.Screen.setCursor(3, 0);
    Controller1.Screen.print("Yjoy = %5.1f ", py);
        
    if (reversed) {
        py *= -1;
    }
    
    double d = sqrt(px*px + py*py);
    double scale = scale_joystick(d);
    
    px *= scale;
    py *= scale;

    double lp = py + px;
    double rp = py - px;

//    Controller1.Screen.setCursor(2, 0);
//    Controller1.Screen.print("Lrpm = %5.1f%%", lp);
//    Controller1.Screen.setCursor(3, 0);
//    Controller1.Screen.print("Rrpm = %5.1f%%", rp);

    for (int i = 0; i < NUM_MOTORS; i++) {
        lmotors[i].spin(vex::directionType::fwd, lp, percentUnits::pct);
        rmotors[i].spin(vex::directionType::fwd, rp, percentUnits::pct);
    }
}

/**
 * Deal with the spinner thing at the front 
 * of the robot. Pls refactor.
 */

static double spinner_rpm = 600.;

void set_spin() {
    if (spinnerOn) {
        Motor05sp.spin(directionType::fwd, spinner_rpm, velocityUnits::rpm);
    } else {
        Motor05sp.stop();
    }
    Controller1.Screen.
}

void spinner_toggle() {
    spinnerOn = !spinnerOn;
    set_spin();
}

void spinner_rpm_up() {
    spinner_rpm *= 1.1;
    set_spin();
}

void spinner_rpm_down() {
    spinner_rpm /= 1.1;
    set_spin();
}

void reverse_toggle() {
    reversed = !reversed;   
}

int main() {
    // Set up action button bindings to functions
    Controller1.ButtonX.pressed(spinner_toggle);
    Controller1.ButtonUp.pressed(spinner_rpm_up);
    Controller1.ButtonDown.pressed(spinner_rpm_down);        
    Controller1.ButtonB.pressed(reverse_toggle);
    
    while(true) {
        // Drive code
        arcadedrive();
    }
}