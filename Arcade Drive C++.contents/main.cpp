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

/* Controller screen lines, if 0, do not print */
const int JOYSTICK_LINE = 1;
const int MOTOR_LINE = 2;
const int SPINNER_LINE = 3;

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
const double DEADZONE = 0.02;
const double JOY_SCALE = 127.0;
static double smooth_power = 0.5;

double scale_joystick(double input)  // input positive between 0 and ~ 1
{
    // result ~= result^(1+smoot_hpower), modulo dead zone
	double result;
	if (input > DEADZONE) {
		result = fmin((input - DEADZONE) / (1.0 - DEADZONE), 1.0);
		result *= pow(result, smooth_power);  // fine control, optional
	}
    else {
        return 0.;
    }
	return result;
}


motor lmotors[] {Motor11dl, Motor01dl, Motor03dl};
motor rmotors[] {Motor04dr, Motor16dr, Motor02dr};
const int NUM_MOTORS = 3; // per side

void arcadedrive() {
    double px = Controller1.Axis1.value();  //Gets the value of the joystick axis on a scale from -127 to 127.
    double py = Controller1.Axis2.value();
        
    double d = sqrt(px*px + py*py) / JOY_SCALE; // distance from the origin, 0 to ~ 1
    double scale = scale_joystick(d);  // rescale that distance
    
    if (JOYSTICK_LINE > 0) {
        // Print joystick and scaling values for information
        Controller1.Screen.setCursor(JOYSTICK_LINE, 0);
        Controller1.Screen.print("J %4.0f %4.0f %s %3.2f", 
                                 py, px, (reversed ? "R" : " "), scale);
    }
    
    if (reversed) {
        py *= -1;
    }
    
    px *= scale / JOY_SCALE;  // rescale to fraction 0 to ~1
    py *= scale / JOY_SCALE;

    // tentative left/right motor power
    double lp = py + px;  // from 0 to ~2
    double rp = py - px;

    // if |motor power| > 1, rescale them both 
    double mapow = fmax(fabs(lp), fabs(rp));
    if (mapow > 1.0) {
        lp /= mapow;
        rp /= mapow;
    }
    
    lp *= 100; // turn into percent, for motor input
    rp *= 100;

    for (int i = 0; i < NUM_MOTORS; i++) {
        lmotors[i].spin(vex::directionType::fwd, lp, percentUnits::pct);
        rmotors[i].spin(vex::directionType::fwd, rp, percentUnits::pct);
    }

    if (MOTOR_LINE > 0) {
        // Print motor values for information
        Controller1.Screen.setCursor(MOTOR_LINE, 0);
        Controller1.Screen.print("M: %6.1f%% %6.1f%%", lp, rp);
    }


}

/**
 * Deal with the spinner thing at the front 
 * of the robot.
 */

static double spinner_rpm = 600.;

void print_spin() {
    if (SPINNER_LINE <= 0) return;  // do nothing
    Controller1.Screen.setCursor(SPINNER_LINE, 0);
    Controller1.Screen.print("S: %s rpm %5.0f", (spinnerOn ? "ON ": "OFF"), spinner_rpm); 
}

void set_spin() {
    if (spinnerOn) {
        Motor05sp.spin(directionType::fwd, spinner_rpm, velocityUnits::rpm);
    } else {
        Motor05sp.stop();
    }
    print_spin();
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
    Controller1.rumble(rumbleCode);
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