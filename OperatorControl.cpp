#include "RobotCode2014.h"

void RobotCode2014::OperatorControl()
{

		// Disable watchdog - user motor safety timer

	GetWatchdog().SetEnabled(false);

	// Set the timeout period for the motors.
	// NEEDS TO BE RE-IMPLEMENTED

	//myEWHSRobot.SetExpiration(MOTOR_EXPIRATION_TIMEOUT_OPERATOR_CONTROL);

		// Clears LCD screen

	myDriverStationLCD->Clear();

		// If compressor is off, turns it on

	if (myCompressor.Enabled() == false) myCompressor.Start();

		// Runs code loop for operator control

	while (IsOperatorControl() && IsEnabled())
	{

			// Assigns values from sensors to variables

		PollSensors();

			// Softens rotational axis of joystick

		AdjustZAxisInput();

			// Drives chassis

		myDriveSystem.MecanumDrive(fltDriveXAxis, fltDriveYAxis, fltNormalizedDriveZAxis);

			// Shoots ball if trigger is pressed, retracts mechanism if not pressed

		Shoot(isDriveTrigger);

			// Controls the angle the arm is at

		ArmControl();

			// Updates LCD screen on driver station

		LCDUpdate();
	}
	myDriveSystem.Disable(); //Stops drive motors
	myArmPID.Disable(); //Stops arm movement
}
