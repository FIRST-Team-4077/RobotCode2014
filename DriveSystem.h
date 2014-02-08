#ifndef DRIVESYSTEM_H_
#define DRIVESYSTEM_H_

#include <WPILib.h>
#include <Math.h>

class DriveSystem
{
public:

	bool isPIDMode, isDistanceInit;

	float fltDistanceTravelled;

	static const float P = 1;
	static const float I = 1;
	static const float D = 0;
	static const float fltPeriod = 0.05;

	Encoder myDistanceEncoder, myLeftFrontDriveEncoder, myLeftRearDriveEncoder, myRightFrontDriveEncoder, myRightRearDriveEncoder;
	Talon myLeftFrontDriveTalon, myLeftRearDriveTalon, myRightFrontDriveTalon, myRightRearDriveTalon;
	PIDController myLeftFrontPIDController, myLeftRearPIDController, myRightFrontPIDController, myRightRearPIDController;

	DriveSystem(UINT32 FrontLeftDriveMotorPort, UINT32 RearLeftDriveMotorPort, UINT32 FrontRightDriveMotorPort, UINT32 RearRightDriveMotorPort, bool isPIDEnable = false);

	void MecanumDrive(float x, float y, float rotation, float gyroAngle = 0.0);

	bool DriveDistance(float fltXDistance, float fltYDistance = 0.0);

	void DirectDrive(float fltLeftFrontESCVelocity, float fltLeftRearESCVelocity, float fltRightFrontESCVelocity, float fltRightRearESCVelocity);

	void RotateVector(double &x, double &y, double angle);

	void Disable();

private:

	UINT32 intFrontLeftDriveMotorPort, intRearLeftDriveMotorPort, intFrontRightDriveMotorPort, intRearRightDriveMotorPort;

};

#endif
