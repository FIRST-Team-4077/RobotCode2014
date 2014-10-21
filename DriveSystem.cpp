#include "DriveSystem.h"

DriveSystem::DriveSystem(Talon *myLeftFrontDriveTalon, Talon *myLeftRearDriveTalon, Talon *myRightFrontDriveTalon, Talon *myRightRearDriveTalon, /*Encoder *myDistanceEncoder,*/ Encoder *myLeftFrontDriveEncoder, Encoder *myLeftRearDriveEncoder, Encoder *myRightFrontDriveEncoder, Encoder *myRightRearDriveEncoder, bool isPIDEnable)
:myLeftFrontPIDController(P, I, D, myLeftFrontDriveEncoder, myLeftFrontDriveTalon, fltPeriod),
myLeftRearPIDController(P, I, D, myLeftRearDriveEncoder, myLeftRearDriveTalon, fltPeriod),
myRightFrontPIDController(P, I, D, myRightFrontDriveEncoder, myRightFrontDriveTalon, fltPeriod),
myRightRearPIDController(P, I, D, myRightRearDriveEncoder, myRightRearDriveTalon, fltPeriod)
{

	myLeftFrontDriveEncoderLocal = myLeftFrontDriveEncoder;
	myLeftRearDriveEncoderLocal = myLeftRearDriveEncoder;
	myRightFrontDriveEncoderLocal = myRightFrontDriveEncoder;
	myRightRearDriveEncoderLocal = myRightRearDriveEncoder;

	myLeftFrontDriveTalonLocal = myLeftFrontDriveTalon;
	myLeftRearDriveTalonLocal = myLeftRearDriveTalon;
	myRightFrontDriveTalonLocal = myRightFrontDriveTalon;
	myRightRearDriveTalonLocal = myRightRearDriveTalon;
		// Enables PID loops to control the drivetrain (sets them to rate, sets range of output, and enables the PID loop)

	if(isPIDEnable)
	{

		myLeftFrontPIDController.SetInputRange(-2300.0, 2300.0);
		myLeftRearPIDController.SetInputRange(-2300.0, 2300.0);
		myRightFrontPIDController.SetInputRange(-2300.0, 2300.0);
		myRightRearPIDController.SetInputRange(-2300.0, 2300.0);

		myLeftFrontPIDController.SetOutputRange(-1.0, 1.0);
		myLeftRearPIDController.SetOutputRange(-1.0, 1.0);
		myRightFrontPIDController.SetOutputRange(-1.0, 1.0);
		myRightRearPIDController.SetOutputRange(-1.0, 1.0);

		myLeftFrontPIDController.Enable();
		myLeftRearPIDController.Enable();
		myRightFrontPIDController.Enable();
		myRightRearPIDController.Enable();
	}

	intFrontLeftDriveMotor = 1;
	intRearLeftDriveMotor = 2;
	intFrontRightDriveMotor = 3;
	intRearRightDriveMotor = 4;

		// Sets global variable distance travelled to zero, distance initialized to false
		// Sets value of PID mode to gloabal variable isPIDMode

	fltDistanceTravelled = 0.0;
	isDistanceInit = false;
	isPIDMode = isPIDEnable;
}

	// Computes motor velocities for use by a mecanum type drivetrain

void DriveSystem::MecanumDrive(float x, float y, float rotation, bool PIDOverride, float gyroAngle)
{
	double xIn = x;
	double yIn = y;
		// Negate y for the joystick.
	yIn = -yIn;
		// Compenstate for gyro angle.
	//RotateVector(xIn, yIn, gyroAngle);

		// Creates an array to store individual motor speeds in, uses fancy math

	double wheelSpeeds[4];
	wheelSpeeds[intFrontLeftDriveMotor] = xIn + yIn + rotation;
	wheelSpeeds[intFrontRightDriveMotor] = -xIn + yIn - rotation;
	wheelSpeeds[intRearLeftDriveMotor] = -xIn + yIn + rotation;
	wheelSpeeds[intRearRightDriveMotor] = xIn + yIn - rotation;

		// Sets the speed of the motors to the calculated speed
		// If PID mode is enabled, calculated speeds are sent to a Pqqqqqqqqqqqqqqqqqqw2211ID controller

	if(PIDOverride == false)
	{
		if(isPIDMode)
		{
			myLeftFrontPIDController.SetPID(1.5, 0.0, 0.1);
			myLeftRearPIDController.SetPID(1.5, 0.0, 0.1);
			myRightFrontPIDController.SetPID(1.5, 0.0, 0.1);
			myRightRearPIDController.SetPID(1.5, 0.0, 0.1);
			myLeftFrontPIDController.Enable();
			myLeftRearPIDController.Enable();
			myRightFrontPIDController.Enable();
			myRightRearPIDController.Enable();
			myLeftFrontPIDController.SetSetpoint(wheelSpeeds[intFrontLeftDriveMotor]);
			myLeftRearPIDController.SetSetpoint(wheelSpeeds[intRearLeftDriveMotor]);
			myRightFrontPIDController.SetSetpoint(-wheelSpeeds[intFrontRightDriveMotor]);
			myRightRearPIDController.SetSetpoint(-wheelSpeeds[intRearRightDriveMotor]);
		} else {
			myLeftFrontDriveTalonLocal->Set(wheelSpeeds[intFrontLeftDriveMotor]);
			myLeftRearDriveTalonLocal->Set(wheelSpeeds[intRearLeftDriveMotor]);
			myRightFrontDriveTalonLocal->Set(-wheelSpeeds[intFrontRightDriveMotor]);
			myRightRearDriveTalonLocal->Set(-wheelSpeeds[intRearRightDriveMotor]);
		}
	} else {
		
		myLeftFrontPIDController.Disable();
		myLeftRearPIDController.Disable();
		myRightFrontPIDController.Disable();
		myRightRearPIDController.Disable();
		
		myLeftFrontDriveTalonLocal->Set(wheelSpeeds[intFrontLeftDriveMotor]);
		myLeftRearDriveTalonLocal->Set(wheelSpeeds[intRearLeftDriveMotor]);
		myRightFrontDriveTalonLocal->Set(-wheelSpeeds[intFrontRightDriveMotor]);
		myRightRearDriveTalonLocal->Set(-wheelSpeeds[intRearRightDriveMotor]);
		/*
		if(x*x > y*y)
		{
			if(x > 0.75)
			{
				// right
				myLeftFrontDriveTalonLocal->Set(1.0);
				myLeftRearDriveTalonLocal->Set(-1.0);
				myRightFrontDriveTalonLocal->Set(1.0);
				myRightRearDriveTalonLocal->Set(-1.0);
			} else if(x < -0.75) {
				// left
				myLeftFrontDriveTalonLocal->Set(-1.0);
				myLeftRearDriveTalonLocal->Set(1.0);
				myRightFrontDriveTalonLocal->Set(-1.0);
				myRightRearDriveTalonLocal->Set(1.0);
			} else {
				myLeftFrontDriveTalonLocal->Set(0.0);
				myLeftRearDriveTalonLocal->Set(0.0);
				myRightFrontDriveTalonLocal->Set(0.0);
				myRightRearDriveTalonLocal->Set(0.0);
			}
		} else {
			if(y > 0.75)
			{
				// backwards
				myLeftFrontDriveTalonLocal->Set(-1.0);
				myLeftRearDriveTalonLocal->Set(-1.0);
				myRightFrontDriveTalonLocal->Set(1.0);
				myRightRearDriveTalonLocal->Set(1.0);
			} else if(y < -0.75) {
				// forwards
				myLeftFrontDriveTalonLocal->Set(1.0);
				myLeftRearDriveTalonLocal->Set(1.0);
				myRightFrontDriveTalonLocal->Set(-1.0);
				myRightRearDriveTalonLocal->Set(-1.0);
			} else {
				myLeftFrontDriveTalonLocal->Set(0.0);
				myLeftRearDriveTalonLocal->Set(0.0);
				myRightFrontDriveTalonLocal->Set(0.0);
				myRightRearDriveTalonLocal->Set(0.0);
			}
		}
	*/
	}
}

	// Drives the robot to a specific distance
	// Horizontal movement is accepted (y direction) but not implemented yet

bool DriveSystem::DriveDistance(float fltXDistance, float fltYDistance)
{
		// Start encoder if not already done
	bool isDistanceReached;
	if(isDistanceInit == false)
	{
			// Set the scaled value for the encoder
		//myDistanceEncoderLocal->SetDistancePerPulse(1);
		//myDistanceEncoderLocal->Start();
	}
		// If the distance needed to drive is greater than the distance driven, the ribit drives forward
		// Utilizes MecanumDrive function
	/*if(myDistanceEncoderLocal->GetDistance() < fltXDistance)
	{
		MecanumDrive(0.5, 0.0, 0.0);
		isDistanceReached = false;
	} else {
		MecanumDrive(0.0, 0.0, 0.0);
		isDistanceReached = true;		
	}*/
		// Returns if the distance has been reached
	return isDistanceReached;
}

	// Drives the motors based on external values
	// Allows the robot to be moved without the mecanum drive mode

void DriveSystem::DirectDrive(float fltLeftFrontESCVelocity, float fltLeftRearESCVelocity, float fltRightFrontESCVelocity, float fltRightRearESCVelocity)
{
	myLeftFrontPIDController.SetSetpoint(fltLeftFrontESCVelocity);
	myLeftRearPIDController.SetSetpoint(fltLeftRearESCVelocity);
	myRightFrontPIDController.SetSetpoint(fltRightFrontESCVelocity);
	myRightRearPIDController.SetSetpoint(fltRightRearESCVelocity);
}

	// Corrects the angle to drive at based on a gyro input using fancy math

void DriveSystem::RotateVector(double &x, double &y, double angle)
{
	double cosA = cos(angle * (3.14159 / 180.0));
	double sinA = sin(angle * (3.14159 / 180.0));
	double xOut = x * cosA - y * sinA;
	double yOut = x * sinA + y * cosA;
	x = xOut;
	y = yOut;
}

	// Disables the talon speed controllers that run the drivetrain

void DriveSystem::Disable()
{
	myLeftFrontDriveTalonLocal->Disable();
	myLeftRearDriveTalonLocal->Disable();
	myRightFrontDriveTalonLocal->Disable();
	myRightRearDriveTalonLocal->Disable();
}
