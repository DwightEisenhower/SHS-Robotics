#include "robot-config.h"

/* Define additional digital outputs.
 * Follow this format:
 * 
 * vex::digital_out <var name> = vex::digital_out( PORT );
 * 
 * Where PORT is a Brain.ThreeWirePort.[letter] = https://www.robotmesh.com/docs/vexv5-cpp/html/classvex_1_1triport.html
 * Ie Brain.ThreeWirePort.A
 */

/* Controller screen lines, if 0, do not print */
const int JOYSTICK_LINE = 1;
const int MOTOR_LINE = 2;
const int SPINNER_LINE = 3;

/**
 * Reversing of motors. If reversed is true then front and back of the robot are reversed.
 */
static bool reversed = false; // Are all motors reversed?

void reverse_toggle() {
    reversed = !reversed;
    Controller1.rumble(".=");
}

/* Joystick rescaling - input^(1+smooth_power) outside the dead zone */
const double DEADZONE = 0.02;
const double JOY_SCALE = 127.0;
static double smooth_power = 0.5;
const double smooth_power_mult = 1.05;

double scale_joystick(double input)  // input positive between 0 and ~ 1
{
    // result ~= input^(1+smooth_power), modulo dead zone
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

void smooth_power_up() {
    smooth_power *= smooth_power_mult;
}

void smooth_power_down() {
    smooth_power /= smooth_power_mult;
}

// Whether to print drive info on controller screen
// (A lot of printing may strain the communication link, so turn it off in competition.)
static bool print_info = false;

void toggle_print_info(){
    print_info = !print_info;
    if (!print_info) {
        // turned off; clear stale info
        if (JOYSTICK_LINE > 0) Controller1.Screen.clearLine(JOYSTICK_LINE);
        if (MOTOR_LINE > 0) Controller1.Screen.clearLine(MOTOR_LINE);
    }
}


// Drive code

motor lmotors[] {Motor11dl, Motor01dl, Motor03dl};
motor rmotors[] {Motor04dr, Motor16dr, Motor02dr};
const int NUM_MOTORS = 3; // per side

void arcadedrive() {
    double px = Controller1.Axis1.value();  //Gets the value of the joystick axis on a scale from -127 to 127.
    double py = Controller1.Axis2.value();
        
    double d = sqrt(px*px + py*py) / JOY_SCALE; // distance from the origin, 0 to ~ 1
    double scale = scale_joystick(d);  // rescale that distance
    
    if (print_info && JOYSTICK_LINE > 0) {
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

    if (print_info && MOTOR_LINE > 0) {
        // Print motor values for information
        Controller1.Screen.setCursor(MOTOR_LINE, 0);
        Controller1.Screen.print("M: %6.1f%% %6.1f%%", lp, rp);
    }
}

/**
 * Deal with the spinner at the front of the robot.
 */
// Spinner states 0 - stopped; 1 - forward; 2 - stopped; 3 - reversed;
// States sycle 0-1-2-3-4-0-... with button press.
static int spinner_state = 0;
static double spinner_rpm = 600.;
const double spinner_rpm_mult = 1.05; 

void print_spin() {
    if (SPINNER_LINE <= 0) return;  // do nothing
    Controller1.Screen.setCursor(SPINNER_LINE, 0);
    Controller1.Screen.print("S: %s rpm %5.0f", (spinner_state % 2 == 0 ? "OFF": (spinner_state == 1 ? "FWD" : "REV")),
                                                 spinner_rpm); 
}

void set_spin() {
    if (spinner_state % 2 == 0) { // states 0 and 2
        Motor05sp.stop();        
    } else {
        Motor05sp.spin((spinner_state == 1 ? directionType::fwd : directionType::rev), spinner_rpm, velocityUnits::rpm);
    }
    print_spin();
}

void spinner_toggle() {
    ++spinner_state %= 4;  // same as spinner_state = (spinner_state + 1) % 4;
    set_spin();
}

void spinner_rpm_up() {
    spinner_rpm *= spinner_rpm_mult;
    set_spin();
}

void spinner_rpm_down() {
    spinner_rpm /= spinner_rpm_mult;
    set_spin();
}

int main() {
    // Set up action button bindings to functions
    Controller1.ButtonX.pressed(spinner_toggle);
    Controller1.ButtonUp.pressed(spinner_rpm_up);
    Controller1.ButtonDown.pressed(spinner_rpm_down);        
    Controller1.ButtonB.pressed(reverse_toggle);
    Controller1.ButtonRight.pressed(smooth_power_up);
    Controller1.ButtonLeft.pressed(smooth_power_down);
    Controller1.ButtonY.pressed(toggle_print_info);
    
    
    while(true) {
        // Drive code
        arcadedrive();
    }
}