#include "RobotCode2014.h"

RobotCode2014::RobotCode2014()
:myArmPotentiometer(ARM_POTENTIOMETER_CHANNEL),
myCompressor(PNEUMATIC_PRESSURE_SWITCH, COMPRESSOR_RELAY),
myPistonOpen(PNEUMATIC_PRESSURE_OPEN),
myPistonClosed(PNEUMATIC_PRESSURE_CLOSED),
myDriveSystem(FRONT_LEFT_DRIVE_MOTOR_PORT, REAR_LEFT_DRIVE_MOTOR_PORT, FRONT_RIGHT_DRIVE_MOTOR_PORT, REAR_RIGHT_DRIVE_MOTOR_PORT),
myPiston1(PISTON1_FORWARD, PISTON1_BACKWARD),
myPiston2(PISTON2_FORWARD, PISTON2_BACKWARD),
myPiston3(PISTON3_FORWARD, PISTON3_BACKWARD),
myPiston4(PISTON4_FORWARD, PISTON4_BACKWARD),
myDriveJoystick(DRIVE_JOYSTICK_PORT),
myArmPID(P, I, D, &myArmPotentiometer, &myArmTalon),
//myLeftFrontMecanumPID(P, I, D, ),
myArmTalon(ARM_TALON_CHANNEL),
myAxisCamera(AxisCamera::GetInstance("10.40.77.11"))
{

		//	Initialize DriverStationLCD 

	myDriverStationLCD = DriverStationLCD::GetInstance();

		//	Let the operator know we are Initializing Robot

	myDriverStationLCD->Clear();
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Initializing Robot");
	myDriverStationLCD->UpdateLCD();

}

START_ROBOT_CLASS(RobotCode2014);
