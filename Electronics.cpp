#include "RobotCode2014.h"

void RobotCode2014::AdjustZAxisInput()
{
	fltNormalizedDriveZAxis = fltDriveZAxis;
	Limit(fltNormalizedDriveZAxis);
	//fltExponent = (fltDriveTwistAxis + 2); Changes fltExponent based on Twist Axis
	fltExponent = 2;
	//fltDeadBand = .2 - ((fltDriveTwistAxis + 1) * .1); Changes fltDeadBand based on Twist Axis
	fltDeadBand = .2;
	if (fltNormalizedDriveZAxis >= 0.0){
		fltNormalizedDriveZAxis = ((1 + fltDeadBand) * fltNormalizedDriveZAxis) - fltDeadBand;
		if (fltNormalizedDriveZAxis <= 0.0)
		{
			fltNormalizedDriveZAxis = 0.0;
		}
		Limit(fltNormalizedDriveZAxis);
		fltNormalizedDriveZAxis = pow(fltNormalizedDriveZAxis, fltExponent);
	} else {
		fltNormalizedDriveZAxis = (- (1 + fltDeadBand) * fltNormalizedDriveZAxis) - fltDeadBand;
		if (fltNormalizedDriveZAxis <= 0.0)
		{
			fltNormalizedDriveZAxis = 0.0;
		}
		Limit(fltNormalizedDriveZAxis);
		fltNormalizedDriveZAxis = - pow(fltNormalizedDriveZAxis, fltExponent);
	}
	fltNormalizedDriveZAxis = 0.75 * fltNormalizedDriveZAxis;
} 

void RobotCode2014::LCDUpdate()
{
	myDriverStationLCD->Clear();
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line1, "X-Axis = %f", fltDriveXAxis);
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Y-Axis = %f", fltDriveYAxis);
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Z-Axis = %f", fltDriveZAxis);
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line4, "Twist-Axis = %f", fltDriveTwistAxis);
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Exponent = %f", fltExponent);
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line6, "DeadBand = %f", fltDeadBand);
	myDriverStationLCD->UpdateLCD();
}

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
