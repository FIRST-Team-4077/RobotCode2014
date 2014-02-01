#include <Joystick.h>
#include <math.h>
#include <types/vxTypes.h>
#include <WPILib.h>
#include <time.h>
#include "DriveSystem.h"

class RobotCode2014 : public SimpleRobot
{
public:

	/*

				LCD Lines

	Line 1 = Null
	Line 2 = Null
	Line 3 = Null
	Line 4 = Null
	Line 5 = Null

				Drive Joystick

	X Axis = Rotational Movement
	Y Axis = Horizontal Movement
	Z Axis = Rotational Movement
	Trigger = Null
	Button 2 = Null
	Button 3 = Null
	Button 4 = Null
	Button 5 = Null
	Button 6 = Null
	Button 7 = Null
	Button 8 = Null
	Button 9 = Null
	Button 10 = Null
	Button 11 = Null
	Button 12 =  Null

	 */

			//  Define PWM Data

	static const uint32_t FRONT_LEFT_DRIVE_MOTOR_PORT = 1;
	static const uint32_t REAR_LEFT_DRIVE_MOTOR_PORT = 2;
	static const uint32_t FRONT_RIGHT_DRIVE_MOTOR_PORT = 3;
	static const uint32_t REAR_RIGHT_DRIVE_MOTOR_PORT = 4;
	static const uint32_t ARM_TALON_CHANNEL = 5;
	//static const uint32_t UNDEFINED = 6;
	//static const uint32_t UNDEFINED = 7;
	//static const uint32_t UNDEFINED = 8;
	//static const uint32_t UNDEFINED = 9;
	//static const uint32_t UNDEFINED = 10;

			//  Define Anolog in Data

	static const uint32_t ARM_POTENTIOMETER_CHANNEL = 1;
	//static const uint32_t UNDEFINED = 2;
	//static const uint32_t UNDEFINED = 3;
	//static const uint32_t UNDEFINED = 4;
	//static const uint32_t UNDEFINED = 5;
	//static const uint32_t UNDEFINED = 6;
	//static const uint32_t UNDEFINED = 7;
	//static const uint32_t UNDEFINED = 8;

			//	Define relay ports

	static const uint32_t COMPRESSOR_RELAY = 1;
	//static const uint32_t UNDEFINED = 2;
	//static const uint32_t UNDEFINED = 3;
	//static const uint32_t UNDEFINED = 4;
	//static const uint32_t UNDEFINED = 5;
	//static const uint32_t UNDEFINED = 6;

			//	Define digital I/O ports

	static const uint32_t PNEUMATIC_PRESSURE_SWITCH = 1;
	static const uint32_t PNEUMATIC_PRESSURE_OPEN = 2;
	static const uint32_t PNEUMATIC_PRESSURE_CLOSED = 3;
	//static const uint32_t UNDEFINED = 4;
	//static const uint32_t UNDEFINED = 5;
	//static const uint32_t UNDEFINED = 6;
	//static const uint32_t UNDEFINED = 7;
	//static const uint32_t UNDEFINED = 8;
	//static const uint32_t UNDEFINED = 9;
	//static const uint32_t UNDEFINED = 10;
	//static const uint32_t UNDEFINED = 11;
	//static const uint32_t UNDEFINED = 12;
	//static const uint32_t UNDEFINED = 13;
	//static const uint32_t UNDEFINED = 14;

			//	Define Solenoid Ports

	static const uint32_t PISTON1_FORWARD = 1;
	static const uint32_t PISTON1_BACKWARD = 2;
	static const uint32_t PISTON2_FORWARD = 3;
	static const uint32_t PISTON2_BACKWARD = 4;
	static const uint32_t PISTON3_FORWARD = 5;
	static const uint32_t PISTON3_BACKWARD = 6;
	static const uint32_t PISTON4_FORWARD = 7;
	static const uint32_t PISTON4_BACKWARD = 8;

	static const uint32_t DRIVE_JOYSTICK_PORT = 1;

			//  PID Variables

	float P;
	float I;
	float D;

			//  Defines when to stop motor if no commands received

	static const float MOTOR_EXPIRATION_TIMEOUT_AUTONOMOUS = 10.0;
	static const float MOTOR_EXPIRATION_TIMEOUT_OPERATOR_CONTROL = 10.0;

	float fltDriveXAxis, fltDriveYAxis, fltDriveZAxis, fltDriveTwistAxis, fltExponent, fltDeadBand, fltNormalizedDriveZAxis;
	
	bool isDriveTrigger, isDriveB2, isDriveB3, isDriveB4, isDriveB5, isDriveB6, isDriveB7, isDriveB8, isDriveB9, isDriveB10, isDriveB11, isDriveB12;

	const char* compressorState;

			//  Define Class Objects

	AnalogChannel				myArmPotentiometer;
	Compressor					myCompressor;
	DigitalInput				myPistonOpen, myPistonClosed;
	DriveSystem					myDriveSystem;
	DoubleSolenoid				myPiston1, myPiston2, myPiston3, myPiston4;
	Joystick					myDriveJoystick;
	PIDController				myArmPID;
	Talon						myArmTalon;
	AxisCamera	 				&myAxisCamera;
	DriverStationLCD			*myDriverStationLCD;


	RobotCode2014();

	void AdjustZAxisInput();//in Electronics

	void Autonomous();//in Autonomous

	void ArmControl();//in Mechanisms

	void LCDUpdate(); //in Electronics

	float Limit(float fltNewValue); //in Electronics

	void OperatorControl(); //in OperatorControl

	void PollSensors();	//in PollSensors

	void Shoot(bool fire); //in Mechanisms

};
