#pragma config(Motor,  port1,           ur_arm,        tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           fr_drive,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           lr_arm,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           rr_drive,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           rl_drive,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           fl_drive,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           weapon,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           ll_arm,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          ul_arm,        tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/******************************************************************
From Radio Control Transmitter/Single Joystick Control.c sample program
-	Parameters and information for vexRT:
Ch1 = Right Joystick X-axis
Full Right = 127
Middle     = 0
Full Left  = -127
Ch2 = Right Joystick Y-axis
Full Up    = 127
Middle     = 0
Full Down  = -127
Ch3 = Left Joystick Y-axis
Full Up    = 127
Middle     = 0
Full Down  = -127
Ch4 = Left Joystick X-axis
Full Right = 127
Middle     = 0
Full Left  = -127
Ch5 = Rear Left Buttons
None Pressed    = 0
Top Pressed     = 127
Down Pressed  	= -127
Both Pressed		= 0
Ch6 = Rear Right Buttons
None Pressed    = 0
Top Pressed     = 127
Down Pressed  	= -127
Both Pressed		= 0
*************************************************************************/

// #define getJoystickValue(channel) vexRT[channel]
//JOY_DRIVE_LV	vexJSLeftV	Vertical axis for the left joystick
//JOY_DRIVE_LH	vexJSLeftH	Horizontal axis for the left joystick
//JOY_DRIVE_RV	vexJSRightV	Vertical axis for the right joystick
//JOY_DRIVE_RH	vexJSRightH	Horizontal axis for the right joystick

//void popper(int blank)
//{
//	if(blank)
//	{
//		motor[weapon] = 100;
//	}
//	if(!blank)
//	{
//		motor[weapon] = 0;
//	}
//}

const int DEADZONE = 10;
const int MAX_MOTOR_VAL = 127;
const float MAX_JOY_VAL = 127.0;
//const float SQRT2 = sqrt(2);

/**
 * Scales a joystick input value (-127 to 127) to a float 
 * between -1 and 1, representing percent motor output.
 * (Negative being reversed). Quadratically scales.
 */
float scaleJoystick(int input) {
    float result = 0;

    /* Input exceeds threshold to function */
    if (abs(input) > DEADZONE) {
        /* Determine a ratio of the input to the x value range 
         * that joystick inputs can have. This ignores the DEADZONE,
         * for example, if my input was 20, graphed on an x-axis, it
         * would look like
         *                         |---------B--------|
         *                     |-C-|-A-|
         * -127 -------------- 0 ---- 20 ---------- 127
         * 
         * The expression below calculates the ratio of A to B, where
         * C is the deadzone (not included). */
        result = (input - DEADZONE) / (MAX_JOY_VAL - DEADZONE);

        /* Quadratically scale the result, making lower input 
         * values grow more slowly than higher ones. At max joystick 
         * input (127) result should be equal to 1, giving max motor
         * strength */
        result *= result;

        /* If input was initially negative make output negative */
        if (input < 0)
            result *= -1;
    } 
}

//float rescale_squares(alpha)
//{
//	float rescale;

//	if (alpha <= PI/4)
//	{
//		rescale = cos(alpha) / (SQRT2 * cos(PI/4 - alpha));	//
//	}
//	else if (alpha <= PI/2)
//	{
//		rescale = cos(PI/2 - alpha) / (SQRT2 * cos(alpha - PI/4));	//case II
//	}
//	else if (alpha <= 3*PI/4)
//	{
//		rescale = cos(alpha - PI/2) / (SQRT2 * cos(3*PI/4 - alpha));  // case I with alpha - PI/2
//	}
//	else if (alpha <= PI)
//	{
//		rescale = cos(PI - alpha) / (SQRT2 * cos(alpha - 3*PI/4)); // case II with alpha - PI/2
//	}
//	return rescale;
//}


void drive()
{
	float v = scale_joystick(getJoystickValue(vexJSRightV));  // vertical
	float h = scale_joystick(getJoystickValue(vexJSRightH));  // horizontal

	//float hv_alpha = fabs(atan2(js_drive_h, js_drive_v));
	float l, r;

	if (v >= 0 && h >= 0)
	{
		l = 0.5 * (v + h + fabs(v-h));
		r = v - h;
	} else if (v < 0 && h < 0)
	{
		l = 0.5 * (v + h - fabs(v-h));
		r = v - h;
	} else if (v >= 0 && h < 0)
	{
		l = v + h;
		r = 0.5 * (v - h + fabs(v + h));
	} else
	{
		l = v + h;
		r =  0.5 * (v - h - fabs(v + h));
	}

	r *= MAX_MOTOR_VAL;
	l *= MAX_MOTOR_VAL;

	motor[fr_drive] = r;
	motor[rr_drive] = r;
	motor[fl_drive] = l;
	motor[rl_drive] = l;
}

void move_arm()
{
	float v = scale_joystick(getJoystickValue(vexJSLeftV));  // vertical
	float h = scale_joystick(getJoystickValue(vexJSLeftH));  // horizontal

	float u, l;

	u = v;
	l = h;

	motor[ll_arm] = l;
	motor[lr_arm] = l;
	motor[ur_arm] = u;
	motor[ul_arm] = u;

}

bool weapon_on = false;

void control_weapon()
{
	if (getJoystickValue(Btn6U))
		weapon_on = true;
	if (getJoystickValue(Btn6D))
		weapon_on = false;

	if (weapon_on)
		motor[weapon] = MAX_MOTOR_VAL;
	else
		motor[weapon] = 0;
}

task main()
{
	while(true)
	{
		drive();
		move_arm();
		control_weapon();
	}
}