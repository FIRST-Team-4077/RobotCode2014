#include "RobotCode2014.h"

void RobotCode2014::PollSensors()
{
	//Joystick Controls

	fltDriveXAxis = myDriveJoystick.GetX();
	fltDriveYAxis = myDriveJoystick.GetY();
	fltDriveZAxis = myDriveJoystick.GetZ();
	fltDriveTwistAxis = myDriveJoystick.GetTwist();

	isDriveTrigger = myDriveJoystick.GetTrigger();
	isDriveB2 = myDriveJoystick.GetRawButton(2);
	isDriveB3 = myDriveJoystick.GetRawButton(3);
	isDriveB4 = myDriveJoystick.GetRawButton(4);
	isDriveB5 = myDriveJoystick.GetRawButton(5);
	isDriveB6 = myDriveJoystick.GetRawButton(6);
	isDriveB7 = myDriveJoystick.GetRawButton(7);
	isDriveB8 = myDriveJoystick.GetRawButton(8);
	isDriveB9 = myDriveJoystick.GetRawButton(9);
	isDriveB10 = myDriveJoystick.GetRawButton(10);
	isDriveB11 = myDriveJoystick.GetRawButton(11);
	isDriveB12 = myDriveJoystick.GetRawButton(12);

	if(myCompressor.Enabled())
	{
		compressorState = "Compressor Running";
	} else {
		compressorState = "Compressor Stopped";
	}
	
}
