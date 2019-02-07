#include "robot-config.h"
#include "debug/debug.h"
#include "chrono.h"
/* Define additional digital outputs.
 * Follow this format:
 * 
 * vex::digital_out <var name> = vex::digital_out( PORT );
 * 
 * Where PORT is a Brain.ThreeWirePort.[letter] = https://www.robotmesh.com/docs/vexv5-cpp/html/classvex_1_1triport.html
 * Ie Brain.ThreeWirePort.A
 */
//Creates a competition object that allows access to Competition methods.
//vex::competition    Competition;
/* Controller screen lines, if 0, do not print */
const int JOYSTICK_LINE = 1;
const int MOTOR_LINE = 2;
const int SPINNER_LINE = 3;

// Whether to print drive info on controller screen
// (A lot of printing may strain the communication link, so turn it off in competition.)
static bool print_info = true;

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

/**
 * Reversing of motors. If reversed is true then front and back of the robot are reversed.
 */
static bool reversed = false; // Are all motors reversed?

// Stopping mode for the motors
// 0 - coast, 1 -brake, 2 - hold
static int stopping_mode_num = 0;
const char stopping_mode_char[] = {'C', 'B', 'H'};
const brakeType stopping_mode[] = {brakeType::coast, brakeType::brake, brakeType::hold};

// Current left/right motor power
static double cur_lp = 0.;
static double cur_rp = 0.;

void print_motor_line() {
    if (print_info && MOTOR_LINE > 0) {
            // Print motor values for information
            Controller1.Screen.setCursor(MOTOR_LINE, 1);
            Controller1.Screen.print("M: %c %c %3.0f%% %3.0f%%     ", (reversed ? 'R' : 'F'), 
                                     stopping_mode_char[stopping_mode_num], cur_lp, cur_rp);
        }
}

void spin_motors(double lp, double rp) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        lmotors[i].spin(vex::directionType::fwd, lp, percentUnits::pct);
        rmotors[i].spin(vex::directionType::fwd, rp, percentUnits::pct);
    }
}

void set_stopping_mode_for_motors(brakeType mode) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        lmotors[i].setStopping(mode);
        rmotors[i].setStopping(mode);
    }
}

void reverse_toggle() {
    reversed = !reversed;
    spin_motors(0., 0.);  // momentarily slow down to 0, so as not be too abrupt 
    print_motor_line();
    Controller1.rumble(".=");
}

void stopping_mode_toggle() {
    ++stopping_mode_num %= 3;  // same as stopping_mode_num = (stopping_mode_num + 1) % 3;
    set_stopping_mode_for_motors(stopping_mode[stopping_mode_num]);
    print_motor_line();
    Controller1.rumble("..");
}

/**
 * Stop all wheel motors 
 */
void stopAllMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        lmotors[i].stop();
        rmotors[i].stop();
    }
}

void stopAllMotors(brakeType bt) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        lmotors[i].stop(bt);
        rmotors[i].stop(bt);
    }
}

void accelerateMotor(motor m, directionType dir) {
    pwr = (m.power()/11)*100; //convert into %
    if(pwr < 100) {
        pwr += 10;
    }
    m.spin(vex::directionType::dir, pwr, percentUnits::pct);
}

void accelerateAllMotors(bool left, directionType dir) {
    if(left) {
        for(int i = 0, i < 3; i++) {
            accelerateMotor(lmotors[i], dir);
        }
    } else {
        for(int i = 0, i < 3; i++) {
            accelerateMotor(rmotors[i], dir);
        }
    }
}

double averagePower(bool left){
    double avg;
    if(left) {
        avg += ( lmotors[0].power() + lmotors[1].power() + lmotors[2].power() ) / 3;
    } else {
        avg += ( lmotors[0].power() + lmotors[1].power() + lmotors[2].power() ) / 3;
    }
}

void GTAdrive() {
    bool fwd = Controller1.ButtonUp.pressing();
    bool rev = Controller1.ButtonDown.pressing();
    bool left = Controller1.ButtonLeft.pressing();
    bool right = Controller1.ButtonRight.pressing();
    bool brake = Controller1.ButtonR1.pressing();
    bool cruise = Controller1.ButtonL1.pressing();//locks motor speeds in place, should help with turns & adjustments
    
    if(!cruise) {
        if(fwd){
            accelerateAllMotors(true, vex::directionType::fwd);
            accelerateAllMotors(false, vex::directionType::fwd);
            vex::task::sleep(50);
        }
        if(rev){
            accelerateAllMotors(true, vex::directionType::fwd);
            accelerateAllMotors(false, vex::directionType::fwd);
            vex::task::sleep(50);
        }
        if(left) {
            accelerateAllMotors(true, vex::directionType::rev);
            vex::task::sleep(100);
        }
        if(right) {
            accelerateAllMotors(false, vex::directionType::rev);
            vex::tast::sleep(100);
        }
    }
    
    double lpwr = averagePower(true);
    double rpwr = averagePower(false);
    double average_power = (lpwr + rpwr) / 2;
    
    //Leveling out after a turn
    if( (fwd || rev) && (!brake && !left && !right && lpwr != rpwr) {
        spin_motors(average_power, average_power);
    }
    if(brake) {
        stopAllMotors(stopping_mode[0]);//may skid but hopefully permits drifting
    }
    
}

/**
 * Deal with the spinner at the front of the robot.
 */
// Spinner states 0 - stopped; 1 - forward; 2 - stopped; 3 - reversed;
// States sycle 0-1-2-3-4-0-... with button press.
static int spinner_state = 0;
static double spinner_rpm = 500.;
const double spinner_rpm_mult = 1.05; 

void print_spin() {
    if (SPINNER_LINE <= 0) return;  // do nothing
    Controller1.Screen.setCursor(SPINNER_LINE, 1);
    Controller1.Screen.print("S: %s rpm %5.0f   ", (spinner_state % 2 == 0 ? "OFF": (spinner_state == 1 ? "FWD" : "REV")),
                                                    spinner_rpm); 
}

void set_spin() {
    if (spinner_state % 2 == 0) { // states 0 and 2
        Motor05sp.stop(brakeType::coast);        
    } else {
        Motor05sp.spin((spinner_state == 1 ? directionType::fwd : directionType::rev), spinner_rpm, velocityUnits::rpm);//if equal to 1, rotate fwd, else rotate rev
    }
}

void spinner_toggle() {
    ++spinner_state %= 4;  // same as spinner_state = (spinner_state + 1) % 4;
    set_spin();
    print_spin();
}

void spinner_rpm_up() {
    spinner_rpm *= spinner_rpm_mult;
    set_spin();
    print_spin();
}

void spinner_rpm_down() {
    spinner_rpm /= spinner_rpm_mult;
    set_spin();
    print_spin();
}


//COMPETITION STUFF


void pre_auton() {
    // Set up action button bindings to functions
    Controller1.ButtonX.pressed(spinner_toggle);
    Controller1.ButtonUp.pressed(spinner_rpm_up);
    Controller1.ButtonDown.pressed(spinner_rpm_down);        
    Controller1.ButtonB.pressed(reverse_toggle);
    Controller1.ButtonRight.pressed(smooth_power_up);
    Controller1.ButtonLeft.pressed(smooth_power_down);
    Controller1.ButtonY.pressed(toggle_print_info);
    Controller1.ButtonA.pressed(stopping_mode_toggle);
    //Controller1.ButtonL1.pressed(autonomous);
    // Set up initial screen
    Controller1.Screen.clearScreen();
    print_motor_line();
    print_spin();
}

void user_control(void){
    while(true) {
        // Drive code
        arcadedrive();
        vex::task::sleep(5); //Sleep the task for a short amount of time to prevent wasted resources. 
    }
}

int main() {
    pre_auton();//setup
    Competition.drivercontrol(user_control);
}

