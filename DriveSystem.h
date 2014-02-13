#ifndef DRIVESYSTEM_H_
#define DRIVESYSTEM_H_

#include <WPILib.h>
#include <Math.h>

class DriveSystem
{
public:

	bool isPIDMode, isDistanceInit;

	float fltDistanceTravelled;

	static const float P = 1.0;
	static const float I = 0.0;
	static const float D = 0.0;
	static const float Pr = -1.0;
	static const float Ir = 0.0;
	static const float Dr = 0.0;
	static const float fltPeriod = 0.05;

	PIDController myLeftFrontPIDController, myLeftRearPIDController, myRightFrontPIDController, myRightRearPIDController;
	Encoder *myDistanceEncoderLocal, *myLeftFrontDriveEncoderLocal, *myLeftRearDriveEncoderLocal, *myRightFrontDriveEncoderLocal, *myRightRearDriveEncoderLocal;
	Talon *myLeftFrontDriveTalonLocal, *myLeftRearDriveTalonLocal, *myRightFrontDriveTalonLocal, *myRightRearDriveTalonLocal;

	DriveSystem(Talon *myLeftFrontDriveTalon, Talon *myLeftRearDriveTalon, Talon *myRightFrontDriveTalon, Talon *myRightRearDriveTalon, Encoder *myDistanceEncoder, Encoder *myLeftFrontDriveEncoder, Encoder *myLeftRearDriveEncoder, Encoder *myRightFrontDriveEncoder, Encoder *myRightRearDriveEncoder, bool isPIDEnable = false);

	void MecanumDrive(float x, float y, float rotation, float gyroAngle = 0.0);

	bool DriveDistance(float fltXDistance, float fltYDistance = 0.0);

	void DirectDrive(float fltLeftFrontESCVelocity, float fltLeftRearESCVelocity, float fltRightFrontESCVelocity, float fltRightRearESCVelocity);

	void RotateVector(double &x, double &y, double angle);

	void Disable();

private:

	UINT32 intFrontLeftDriveMotor, intRearLeftDriveMotor, intFrontRightDriveMotor, intRearRightDriveMotor;

};

#endif
