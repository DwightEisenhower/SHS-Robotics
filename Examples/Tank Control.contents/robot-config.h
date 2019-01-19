using namespace vex;
vex::brain Brain;
vex::motor RightMotor (vex::PORT1, vex::gearSetting::ratio18_1,false);
vex::motor RightMotor2 (vex::PORT2, vex::gearSetting::ratio18_1,false);
vex::motor ClawMotor (vex::PORT9, vex::gearSetting::ratio18_1,false);
vex::motor LeftMotor3 (vex::PORT10, vex::gearSetting::ratio18_1,true);
vex::motor RightMotor1 (vex::PORT12, vex::gearSetting::ratio18_1,false);
vex::motor LeftMotor1 (vex::PORT20, vex::gearSetting::ratio18_1,false);
vex::controller Controller1;