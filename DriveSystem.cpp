#include "DriveSystem.h"

DriveSystem::DriveSystem(UINT32 FrontLeftDriveMotorPort, UINT32 RearLeftDriveMotorPort, UINT32 FrontRightDriveMotorPort, UINT32 RearRightDriveMotorPort)
:myLeftFrontDriveTalon(FrontLeftDriveMotorPort),
myLeftRearDriveTalon(RearLeftDriveMotorPort),
myRightFrontDriveTalon(FrontRightDriveMotorPort),
myRightRearDriveTalon(RearRightDriveMotorPort)
{
	intFrontLeftDriveMotorPort = FrontLeftDriveMotorPort;
	intRearLeftDriveMotorPort = RearLeftDriveMotorPort;
	intFrontRightDriveMotorPort = FrontRightDriveMotorPort;
	intRearRightDriveMotorPort = RearRightDriveMotorPort;
}

void DriveSystem::MecanumDrive(float x, float y, float rotation, float gyroAngle)
{

	double xIn = x;
	double yIn = y;
	// Negate y for the joystick.
	yIn = -yIn;
	// Compenstate for gyro angle.
	RotateVector(xIn, yIn, gyroAngle);

	double wheelSpeeds[4];
	wheelSpeeds[intFrontLeftDriveMotorPort] = xIn + yIn + rotation;
	wheelSpeeds[intFrontRightDriveMotorPort] = -xIn + yIn - rotation;
	wheelSpeeds[intRearLeftDriveMotorPort] = -xIn + yIn + rotation;
	wheelSpeeds[intRearRightDriveMotorPort] = xIn + yIn - rotation;

	myLeftFrontDriveTalon.Set(wheelSpeeds[intFrontLeftDriveMotorPort] * 1);
	myRightFrontDriveTalon.Set(wheelSpeeds[intFrontRightDriveMotorPort] * -1);
	myLeftRearDriveTalon.Set(wheelSpeeds[intRearLeftDriveMotorPort] * 1);
	myRightRearDriveTalon.Set(wheelSpeeds[intRearRightDriveMotorPort] * -1);
}

void DriveSystem::DirectDrive(float fltLeftFrontESCVelocity, float fltLeftRearESCVelocity, float fltRightFrontESCVelocity, float fltRightRearESCVelocity)
{
	myLeftFrontDriveTalon.Set(fltLeftFrontESCVelocity);
	myLeftRearDriveTalon.Set(fltLeftRearESCVelocity);
	myRightFrontDriveTalon.Set(fltRightFrontESCVelocity);
	myRightRearDriveTalon.Set(fltRightRearESCVelocity);
}

void DriveSystem::RotateVector(double &x, double &y, double angle)
{
	double cosA = cos(angle * (3.14159 / 180.0));
	double sinA = sin(angle * (3.14159 / 180.0));
	double xOut = x * cosA - y * sinA;
	double yOut = x * sinA + y * cosA;
	x = xOut;
	y = yOut;
}

void DriveSystem::StopMotor()
{
	myLeftFrontDriveTalon.Disable();
	myLeftRearDriveTalon.Disable();
	myRightFrontDriveTalon.Disable();
	myRightRearDriveTalon.Disable();
}
