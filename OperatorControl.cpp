#include "RobotCode2014.h"

void RobotCode2014::OperatorControl()
{

	//Disable watchdog - user motor safety timer

	GetWatchdog().SetEnabled(false);	

	//Set the timeout period for the motors.

	myEWHSRobot.SetExpiration(MOTOR_EXPIRATION_TIMEOUT_OPERATOR_CONTROL); 
	myEWHSRobot.SetSafetyEnabled(false); //Enable the motor safety feature

	myDriverStationLCD->Clear();

	while (IsOperatorControl() && IsEnabled())
	{
		myCompressor.Start();

		PollSensors();

		AdjustZAxisInput();

		myEWHSRobot.MecanumDrive_Cartesian(fltDriveXAxis, fltDriveYAxis, fltNormalizedDriveZAxis);
		//myEWHSRobot.ArcadeDrive(fltDriveXAxis, fltDriveYAxis);

		Shoot(isDriveTrigger);

		LCDUpdate();
	}
	myEWHSRobot.Drive(0.0, 0.0); // drive 0% forward speed with 0% turn (stop)
}
