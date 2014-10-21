#include "RobotCode2014.h"

RobotCode2014::RobotCode2014()
:myArmPotentiometer(ARM_POTENTIOMETER_CHANNEL),
myCompressor(PNEUMATIC_PRESSURE_SWITCH, COMPRESSOR_RELAY),
myPiston1(PISTON1_FORWARD, PISTON1_BACKWARD),
myPiston2(PISTON2_FORWARD, PISTON2_BACKWARD),
myPiston3(PISTON3_FORWARD, PISTON3_BACKWARD),
myPiston4(PISTON4_FORWARD, PISTON4_BACKWARD),
myLeftFrontDriveEncoder(ENCODER_1_CHANNEL_A, ENCODER_1_CHANNEL_B, true, Encoder::k4X),
myLeftRearDriveEncoder(ENCODER_2_CHANNEL_A, ENCODER_2_CHANNEL_B, true, Encoder::k4X),
myRightFrontDriveEncoder(ENCODER_3_CHANNEL_A, ENCODER_3_CHANNEL_B, false, Encoder::k4X),
myRightRearDriveEncoder(ENCODER_4_CHANNEL_A, ENCODER_4_CHANNEL_B, false, Encoder::k4X),
myDriveJoystick(DRIVE_JOYSTICK_PORT),
myArmPID(fltArmP, fltArmI, fltArmD, &myArmPotentiometer, &myArmTalon, 0.02),
myCameraServo(CAMERA_SERVO_CHANNEL),
myArmTalon(ARM_TALON_CHANNEL),
myWheelTalon(WHEEL_TALON_CHANNEL),
myLeftFrontDriveTalon(FRONT_LEFT_DRIVE_MOTOR_PORT),
myLeftRearDriveTalon(REAR_LEFT_DRIVE_MOTOR_PORT),
myRightFrontDriveTalon(FRONT_RIGHT_DRIVE_MOTOR_PORT),
myRightRearDriveTalon(REAR_RIGHT_DRIVE_MOTOR_PORT),
myAutonomousTimer(),
myArmTimer(),
myShootTimer(),
myDriveSystem(&myLeftFrontDriveTalon, &myLeftRearDriveTalon, &myRightFrontDriveTalon, &myRightRearDriveTalon, &myLeftFrontDriveEncoder, &myLeftRearDriveEncoder, &myRightFrontDriveEncoder, &myRightRearDriveEncoder, true),
//myThreshold(105, 137, 230, 255, 133, 183) //from sample code
//myThreshold(88, 156, 85, 255, 85, 255) //best from home samples #1 WORKED BUT MANY TGTS
myThreshold(49, 151, 136, 255, 109, 255), //best from home samples #2 WORKED BEST 2TGTS ONLY; WORKS WITH MOST 2014SAMPLES
//myThreshold(64, 173, 68, 255, 109, 255) //from 2012 code/best for 2014Samples
myAxisCamera(AxisCamera::GetInstance("10.40.77.11"))
{

	//	Initialize Camera Variables - from 2012 code
	//Resolution_t {kResolution_640x480, kResolution_640x360, kResolution_320x240, kResolution_160x120};
	myAxisCamera.WriteResolution(AxisCameraParams::kResolution_640x480);
	myAxisCamera.WriteCompression(30); //recommended in white paper
	myAxisCamera.WriteBrightness(30); // filters out some of the bright light. (also used 40,65)
	myAxisCamera.WriteWhiteBalance(AxisCameraParams::kWhiteBalance_Automatic);

	myCameraServo.SetAngle(110.0);
	foundHotTarget = false;

		// Set encoder type to rate (not position)

	myLeftFrontDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);
	myLeftRearDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);
	myRightFrontDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);
	myRightRearDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);

	intArmPosition = 0;
	isArmLoop = false;
	isShootLoop = false;
	intShootingMode = 0; // 0 = Strong Shooting Mode, 1 = Weak Sooting Mode
	isShootLockout = true; // Prevents robot from shooting

		// Sets parameters for the arm PID loop

	myArmPID.SetInputRange(0.0, 1024.0);
	myArmPID.SetOutputRange(-0.5, 0.5);
	myArmPotentiometer.SetAverageBits(3);

		// Initialize driver station LCD 

	myDriverStationLCD = DriverStationLCD::GetInstance();

	isAutonomous = true;

		// Let the operator know we are Initializing Robot

	myDriverStationLCD->Clear();
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Initializing Robot");
	myDriverStationLCD->UpdateLCD();

}

START_ROBOT_CLASS(RobotCode2014);
