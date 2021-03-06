#include "robot-config.h"

/* Define additional digital outputs.
 * Follow this format:
 * 
 * vex::digital_out <var name> = vex::digital_out( PORT );
 * 
 * Where PORT is a Brain.ThreeWirePort.[letter] = https://www.robotmesh.com/docs/vexv5-cpp/html/classvex_1_1triport.html
 * Ie Brain.ThreeWirePort.A
 */
//Creates a competition object that allows access to Competition methods.
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
    double pwr = (m.power(powerUnits::watt)/11)*100; //convert into %
    if(pwr < 100) {
        pwr += 10;
    }
    m.spin(dir, pwr, percentUnits::pct);
}

void accelerateAllMotors(bool left, directionType dir) {
    if(left) {
        for(int i = 0; i < 3; i++) {
            accelerateMotor(lmotors[i], dir);
        }
    } else {
        for(int i = 0; i < 3; i++) {
            accelerateMotor(rmotors[i], dir);
        }
    }
}

double averagePower(bool left){
    double avg = 0.0;
    if(left) {
        avg += ( lmotors[0].power(powerUnits::watt) + lmotors[1].power(powerUnits::watt) + lmotors[2].power(powerUnits::watt) ) / 3;
    } else {
        avg += ( lmotors[0].power(powerUnits::watt) + lmotors[1].power(powerUnits::watt) + lmotors[2].power(powerUnits::watt) ) / 3;
    }
    return avg;
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
            vex::task::sleep(100);
        }
    }
    
    double lpwr = averagePower(true);
    double rpwr = averagePower(false);
    double average_power = (lpwr + rpwr) / 2;
    
    //Leveling out after a turn
    if( (fwd || rev) && (!brake && !left && !right && lpwr != rpwr)) {
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

/**
 * Autonomous methods
 */
int autonState = 1;
/**
 * moveStraight with power, forward direction and time
 *
 * @param power=100  Power
 * @param fwd=true   Go forward?
 * @param time=1000  Delay in ms before stopping
 */
void moveStraight (int power=100, bool fwd=true, int time=1000) {
    vex::directionType dir = fwd ? vex::directionType::fwd : vex::directionType::rev;
    for (int i = 0; i < NUM_MOTORS; i++) {
        lmotors[i].spin(dir, power, percentUnits::pct);
        rmotors[i].spin(dir, power, percentUnits::pct);
    }
    
    task::sleep(time);
    set_stopping_mode_for_motors(hold);
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
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        lmotors[i].spin(dirL, 100, percentUnits::pct);
        rmotors[i].spin(dirR, 100, percentUnits::pct);
    }
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
    moveStraightDistance(1.65);
    task::sleep(100);
    moveStraightDistance(2.3, false);
    task::sleep(100);
    rotate(isRed ? -90 : 90);
    moveStraightDistance(1.8);
}

/**
 * Auton: Rotate 35 degrees then go forward onto
 * the platform 
 */
void auton34(bool isRed) {
    rotate(isRed ? -90 : 90);
    moveStraightDistance(0.6);
    rotate(isRed ? 90 : -90);
    moveStraightDistance(1.3);
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
            Brain.Screen.print("Face robot spinner towards flag");
            break;
        case 2:
            Brain.Screen.setCursor(1,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("A2 - Blue flag");
            
            Brain.Screen.setCursor(2,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("Face robot spinner towards from flag");
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
		case 5:
            Brain.Screen.setCursor(1,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("A5 - AUTON disabled");
            
            Brain.Screen.setCursor(2,0);
            Brain.Screen.clearLine();
            Brain.Screen.print("Robot will now do nothing");
    }
    Brain.Screen.render();
}

/**
 * Runs when screen is pressed. Toggles the
 * auton state */
void screenpressed(void) {
    autonState = autonState % 5 + 1;
    displayCurrentAutonState();
    
    Brain.Screen.setCursor(3,0);
    Brain.Screen.clearLine();
    Brain.Screen.print("Press screen to toggle auton state");
}


void autonomous(void){
    switch (autonState) {
        case 1: auton12(true);
                break;
        case 2: auton12(false);
                break;
        case 3: auton34(true);
                break;
        case 4: auton34(false);
                break;
		case 5: break;
    }
}



void pre_auton() {
    // Set up action button bindings to scoring functions
    Controller1.ButtonX.pressed(spinner_toggle);       
    Controller1.ButtonA.pressed(*reverse_toggle);
    Controller1.ButtonY.pressed(toggle_print_info);
    Controller1.ButtonB.pressed(stopping_mode_toggle);
    
    //Set up action button bindings for driving
    //Controller1.ButtonUp.pressed(*GTAdrive());
    //Controller1.ButtonDown.pressed(GTAdrive());
    //Controller1.ButtonLeft.pressed(GTAdrive());
    //Controller1.ButtonRight.pressed(GTAdrive());
    //Controller1.ButtonL1.pressed(GTAdrive());
    //Controller1.ButtonR1.pressed(GTAdrive());
    
    //Controller1.ButtonL2.pressed(autonomous);
    // Set up initial screen
    Controller1.Screen.clearScreen();
    print_motor_line();
    print_spin();

    /* Display initial text */
    displayCurrentAutonState();
    Brain.Screen.setCursor(3,0);
    Brain.Screen.clearLine();
    Brain.Screen.print("Press screen to toggle auton state");
}

void user_control(void){
    while(true) {
        // Drive code
        GTAdrive();
        vex::task::sleep(5); //Sleep the task for a short amount of time to prevent wasted resources. 
    }
}

int main() {
    pre_auton();//setup
	Competition.autonomous(autonomous);
    Competition.drivercontrol(user_control);
}

