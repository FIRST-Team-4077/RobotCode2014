#include "RobotCode2014.h"

RobotCode2014::RobotCode2014()
:myArmPotentiometer(ARM_POTENTIOMETER_CHANNEL),
myCompressor(PNEUMATIC_PRESSURE_SWITCH, COMPRESSOR_RELAY),
myPistonOpen(PNEUMATIC_PRESSURE_OPEN),
myPistonClosed(PNEUMATIC_PRESSURE_CLOSED),
myDriveSystem(FRONT_LEFT_DRIVE_MOTOR_PORT, REAR_LEFT_DRIVE_MOTOR_PORT, FRONT_RIGHT_DRIVE_MOTOR_PORT, REAR_RIGHT_DRIVE_MOTOR_PORT, true),
myPiston1(PISTON1_FORWARD, PISTON1_BACKWARD),
myPiston2(PISTON2_FORWARD, PISTON2_BACKWARD),
myPiston3(PISTON3_FORWARD, PISTON3_BACKWARD),
myPiston4(PISTON4_FORWARD, PISTON4_BACKWARD),
myDriveJoystick(DRIVE_JOYSTICK_PORT),
myArmPID(fltArmP, fltArmI, fltArmD, &myArmPotentiometer, &myArmTalon, 0.03),
myArmTalon(ARM_TALON_CHANNEL),
myWheelTalon(WHEEL_TALON_CHANNEL),
myAutonomousTimer(),
myAxisCamera(AxisCamera::GetInstance("10.40.77.11"))
{

	intArmPosition = 0;
	isArmLoop = false;

		// Sets parameters for the arm PID loop

	myArmPID.SetInputRange(0.0, 1024.0);
	myArmPID.SetOutputRange(-1.0, 1.0);
	myArmPotentiometer.SetAverageBits(3);

		// Initialize driver station LCD 

	myDriverStationLCD = DriverStationLCD::GetInstance();

		// Let the operator know we are Initializing Robot

	myDriverStationLCD->Clear();
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Initializing Robot");
	myDriverStationLCD->UpdateLCD();

}

START_ROBOT_CLASS(RobotCode2014);
