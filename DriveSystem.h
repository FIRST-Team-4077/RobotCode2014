#ifndef DRIVESYSTEM_H_
#define DRIVESYSTEM_H_

#include <WPILib.h>
#include <Math.h>

class DriveSystem
{
public:

	Talon myLeftFrontDriveTalon, myLeftRearDriveTalon, myRightFrontDriveTalon, myRightRearDriveTalon;

	DriveSystem(UINT32 FrontLeftDriveMotorPort, UINT32 RearLeftDriveMotorPort, UINT32 FrontRightDriveMotorPort, UINT32 RearRightDriveMotorPort);

	void MecanumDrive(float x, float y, float rotation, float gyroAngle = 0.0);

	void DirectDrive(float fltLeftFrontESCVelocity, float fltLeftRearESCVelocity, float fltRightFrontESCVelocity, float fltRightRearESCVelocity);

	void RotateVector(double &x, double &y, double angle);

	void StopMotor();

private:

	UINT32 intFrontLeftDriveMotorPort, intRearLeftDriveMotorPort, intFrontRightDriveMotorPort, intRearRightDriveMotorPort;

};

#endif
