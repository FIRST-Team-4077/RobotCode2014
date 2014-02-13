#include "RobotCode2014.h"

RobotCode2014::RobotCode2014()
:myArmPotentiometer(ARM_POTENTIOMETER_CHANNEL),
myCompressor(PNEUMATIC_PRESSURE_SWITCH, COMPRESSOR_RELAY),
myPiston1(PISTON1_FORWARD, PISTON1_BACKWARD),
myPiston2(PISTON2_FORWARD, PISTON2_BACKWARD),
myPiston3(PISTON3_FORWARD, PISTON3_BACKWARD),
myPiston4(PISTON4_FORWARD, PISTON4_BACKWARD),
myDistanceEncoder(ENCODER_1_CHANNEL_A, ENCODER_1_CHANNEL_B, false, Encoder::k4X),
myLeftFrontDriveEncoder(ENCODER_1_CHANNEL_A, ENCODER_1_CHANNEL_B, true, Encoder::k4X),
myLeftRearDriveEncoder(ENCODER_2_CHANNEL_A, ENCODER_2_CHANNEL_B, true, Encoder::k4X),
myRightFrontDriveEncoder(ENCODER_3_CHANNEL_A, ENCODER_3_CHANNEL_B, false, Encoder::k4X),
myRightRearDriveEncoder(ENCODER_4_CHANNEL_A, ENCODER_4_CHANNEL_B, false, Encoder::k4X),
myDriveJoystick(DRIVE_JOYSTICK_PORT),
myArmPID(fltArmP, fltArmI, fltArmD, &myArmPotentiometer, &myArmTalon, 0.03),
myArmTalon(ARM_TALON_CHANNEL),
myWheelTalon(WHEEL_TALON_CHANNEL),
myLeftFrontDriveTalon(FRONT_LEFT_DRIVE_MOTOR_PORT),
myLeftRearDriveTalon(REAR_LEFT_DRIVE_MOTOR_PORT),
myRightFrontDriveTalon(FRONT_RIGHT_DRIVE_MOTOR_PORT),
myRightRearDriveTalon(REAR_RIGHT_DRIVE_MOTOR_PORT),
myAutonomousTimer(),
myShootTimer(),
myDriveSystem(&myLeftFrontDriveTalon, &myLeftRearDriveTalon, &myRightFrontDriveTalon, &myRightRearDriveTalon, &myDistanceEncoder, &myLeftFrontDriveEncoder, &myLeftRearDriveEncoder, &myRightFrontDriveEncoder, &myRightRearDriveEncoder, false),
myAxisCamera(AxisCamera::GetInstance("10.40.77.11"))
{

		// Set encoder type to rate (not position)

	myLeftFrontDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);
	myLeftRearDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);
	myRightFrontDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);
	myRightRearDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);

	myLeftFrontDriveEncoder.Start();
	myLeftRearDriveEncoder.Start();
	myRightFrontDriveEncoder.Start();
	myRightRearDriveEncoder.Start();


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
