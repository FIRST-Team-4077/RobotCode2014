#include "DriveSystem.h"

DriveSystem::DriveSystem(UINT32 FrontLeftDriveMotorPort, UINT32 RearLeftDriveMotorPort, UINT32 FrontRightDriveMotorPort, UINT32 RearRightDriveMotorPort, bool isPIDEnable)
:myDistanceEncoder(9, 10, false, Encoder::k4X),
myLeftFrontDriveEncoder(1, 2, false, Encoder::k4X),
myLeftRearDriveEncoder(3, 4, false, Encoder::k4X),
myRightFrontDriveEncoder(5, 6, false, Encoder::k4X),
myRightRearDriveEncoder(7, 8, false, Encoder::k4X),
myLeftFrontDriveTalon(FrontLeftDriveMotorPort),
myLeftRearDriveTalon(RearLeftDriveMotorPort),
myRightFrontDriveTalon(FrontRightDriveMotorPort),
myRightRearDriveTalon(RearRightDriveMotorPort),
myLeftFrontPIDController(P, I, D, &myLeftFrontDriveEncoder, &myLeftFrontDriveTalon, fltPeriod),
myLeftRearPIDController(P, I, D, &myLeftRearDriveEncoder, &myLeftRearDriveTalon, fltPeriod),
myRightFrontPIDController(P, I, D, &myRightFrontDriveEncoder, &myRightFrontDriveTalon, fltPeriod),
myRightRearPIDController(P, I, D, &myRightRearDriveEncoder, &myRightRearDriveTalon, fltPeriod)
{

		// Enables PID loops to control the drivetrain (sets them to rate, sets range of output, and enables the PID loop)

	if(isPIDEnable)
	{
		myLeftFrontDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);
		myLeftRearDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);
		myRightFrontDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);
		myRightRearDriveEncoder.SetPIDSourceParameter(PIDSource::kRate);

		myLeftFrontPIDController.SetOutputRange(-1.0, 1.0);
		myLeftFrontPIDController.SetOutputRange(-1.0, 1.0);
		myLeftFrontPIDController.SetOutputRange(-1.0, 1.0);
		myLeftFrontPIDController.SetOutputRange(-1.0, 1.0);

		myLeftFrontPIDController.Enable();
		myLeftRearPIDController.Enable();
		myRightFrontPIDController.Enable();
		myRightRearPIDController.Enable();
	}

	intFrontLeftDriveMotorPort = FrontLeftDriveMotorPort;
	intRearLeftDriveMotorPort = RearLeftDriveMotorPort;
	intFrontRightDriveMotorPort = FrontRightDriveMotorPort;
	intRearRightDriveMotorPort = RearRightDriveMotorPort;

		// Sets global variable distance travelled to zero, distance initialized to false
		// Sets value of PID mode to gloabal variable isPIDMode

	fltDistanceTravelled = 0;
	isDistanceInit = false;
	isPIDMode = isPIDEnable;
}

	// Computes motor velocities for use by a mecanum type drivetrain

void DriveSystem::MecanumDrive(float x, float y, float rotation, float gyroAngle)
{
	double xIn = x;
	double yIn = y;
		// Negate y for the joystick.
	yIn = -yIn;
		// Compenstate for gyro angle.
	RotateVector(xIn, yIn, gyroAngle);

		// Creates an array to store individual motor speeds in, uses fancy math

	double wheelSpeeds[4];
	wheelSpeeds[intFrontLeftDriveMotorPort] = xIn + yIn + rotation;
	wheelSpeeds[intFrontRightDriveMotorPort] = -xIn + yIn - rotation;
	wheelSpeeds[intRearLeftDriveMotorPort] = -xIn + yIn + rotation;
	wheelSpeeds[intRearRightDriveMotorPort] = xIn + yIn - rotation;

		// Sets the speed of the motors to the calculated speed
		// If PID mode is enabled, calculated speeds are sent to a PID controller

	if(isPIDMode)
	{
		myLeftFrontPIDController.SetSetpoint(wheelSpeeds[intFrontLeftDriveMotorPort]);
		myLeftRearPIDController.SetSetpoint(wheelSpeeds[intRearLeftDriveMotorPort]);
		myRightFrontPIDController.SetSetpoint(wheelSpeeds[intFrontRightDriveMotorPort]);
		myRightRearPIDController.SetSetpoint(wheelSpeeds[intRearRightDriveMotorPort]);
	}
	else
	{
		myLeftFrontDriveTalon.Set(wheelSpeeds[intFrontLeftDriveMotorPort]);
		myRightFrontDriveTalon.Set(-wheelSpeeds[intFrontRightDriveMotorPort]);
		myLeftRearDriveTalon.Set(wheelSpeeds[intRearLeftDriveMotorPort]);
		myRightRearDriveTalon.Set(-wheelSpeeds[intRearRightDriveMotorPort]);
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
		myDistanceEncoder.SetDistancePerPulse(1);
		myDistanceEncoder.Start();
	}
		// If the distance needed to drive is greater than the distance driven, the ribit drives forward
		// Utilizes MecanumDrive function
	if(myDistanceEncoder.GetDistance() < fltXDistance)
	{
		MecanumDrive(0.5, 0.0, 0.0);
		isDistanceReached = false;
	} else {
		MecanumDrive(0.0, 0.0, 0.0);
		isDistanceReached = true;		
	}
		// Returns if the distance has been reached
	return isDistanceReached;
}

	// Drives the motors based on external values
	// Allows the robot to be moved without the mecanum drive mode

void DriveSystem::DirectDrive(float fltLeftFrontESCVelocity, float fltLeftRearESCVelocity, float fltRightFrontESCVelocity, float fltRightRearESCVelocity)
{
	myLeftFrontDriveTalon.Set(fltLeftFrontESCVelocity);
	myLeftRearDriveTalon.Set(fltLeftRearESCVelocity);
	myRightFrontDriveTalon.Set(fltRightFrontESCVelocity);
	myRightRearDriveTalon.Set(fltRightRearESCVelocity);
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
	myLeftFrontDriveTalon.Disable();
	myLeftRearDriveTalon.Disable();
	myRightFrontDriveTalon.Disable();
	myRightRearDriveTalon.Disable();
}
