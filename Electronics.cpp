#include "RobotCode2014.h"

	// Softens rotational movement by adding in a deadband and squaring the joystick's y axis
	// v Fany math stuff v

void RobotCode2014::AdjustZAxisInput()
{
	fltNormalizedDriveZAxis = fltDriveZAxis;
	Limit(fltNormalizedDriveZAxis);
	//fltExponent = (fltDriveTwistAxis + 2); Changes fltExponent based on Twist Axis
	fltExponent = 3;
	//fltDeadBand = .2 - ((fltDriveTwistAxis + 1) * .1); Changes fltDeadBand based on Twist Axis
	fltDeadBand = .1;
	if(fltNormalizedDriveZAxis >= 0.0){
		fltNormalizedDriveZAxis = ((1 + fltDeadBand) * fltNormalizedDriveZAxis) - fltDeadBand;
		if (fltNormalizedDriveZAxis <= 0.0)
		{
			fltNormalizedDriveZAxis = 0.0;
		}
		Limit(fltNormalizedDriveZAxis);
		fltNormalizedDriveZAxis = pow(fltNormalizedDriveZAxis, fltExponent);
	} else {
		fltNormalizedDriveZAxis = (- (1 + fltDeadBand) * fltNormalizedDriveZAxis) - fltDeadBand;
		if(fltNormalizedDriveZAxis <= 0.0)
		{
			fltNormalizedDriveZAxis = 0.0;
		}
		Limit(fltNormalizedDriveZAxis);
		fltNormalizedDriveZAxis = - pow(fltNormalizedDriveZAxis, fltExponent);
	}
	//if
	fltNormalizedDriveZAxis = 0.4 * fltNormalizedDriveZAxis;
}

		// Clears then updates the LCD screen displayed on the driver station

void RobotCode2014::LCDUpdate()
{
	float i = myArmPotentiometer.GetValue();
	myDriverStationLCD->Clear();
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Arm Angle = %f", fltArmAngle);
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Arm Pot = %f", i);
	myDriverStationLCD->UpdateLCD();
}

	// Limits a value inputted to this function to be between -1 and 1

float RobotCode2014::Limit(float fltInput)
{
	if (fltInput > 1.0)
	{
		fltInput = 1.0;
	}
	else if (fltInput < -1.0) 
	{
		fltInput = -1.0;
	}
	return fltInput;
}

	// Reads all inputs and assigns them to variables

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

	fltArmAngle = (90.0 / 320.0) * (myArmPotentiometer.GetValue() - 530.0);

	if(myCompressor.Enabled())
	{
		compressorState = "Compressor Running";
	} else {
		compressorState = "Compressor Stopped";
	}

	fltShootTimer = myShootTimer.Get();
}
