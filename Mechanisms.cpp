#include "RobotCode2014.h"

void RobotCode2014::Shoot(bool trigger)
{
	if(isDriveB12)
	{
		myWheelTalon.Set(0.5);
		intShootingMode = 0;
		isShootLockout = false;
	} else if(isDriveB10) {
		myWheelTalon.Set(0.5);
		intShootingMode = 1;
		isShootLockout = false;
	} else if(isAutonomous){
		myWheelTalon.Set(0.5);
		intShootingMode = 0;
		isShootLockout = false;
	} else {
		isShootLockout = true;
	}

	if(intShootingMode == 0) {
		if(trigger == true && fltShootTimer == 0.0 && isShootLoop == false && isShootLockout == false)
		{
			myPiston1.Set(DoubleSolenoid::kForward);
			myPiston2.Set(DoubleSolenoid::kForward);
			myPiston3.Set(DoubleSolenoid::kForward);
			myPiston4.Set(DoubleSolenoid::kForward);
			myShootTimer.Start();
			isShootLoop = true;
			myDriverStationLCD->Clear();
			myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Hell Yeah");
			myDriverStationLCD->UpdateLCD();
		} else if (fltShootTimer > 0.3) {
			myPiston1.Set(DoubleSolenoid::kReverse);
			myPiston2.Set(DoubleSolenoid::kReverse);
			myPiston3.Set(DoubleSolenoid::kReverse);
			myPiston4.Set(DoubleSolenoid::kReverse);
			myShootTimer.Stop();
			myShootTimer.Reset();
			intShootingMode = 0;
			myDriverStationLCD->Clear();
			myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Naw");
			myDriverStationLCD->UpdateLCD();
		} else if(trigger == false && isShootLoop == true) {
			isShootLoop = false;
		}
	} else if(intShootingMode == 1) {
		if(trigger == true && fltShootTimer == 0.0 && isShootLoop == false && isShootLockout == false)
		{
			myPiston1.Set(DoubleSolenoid::kForward);
			myPiston2.Set(DoubleSolenoid::kForward);
			myPiston3.Set(DoubleSolenoid::kForward);
			myPiston4.Set(DoubleSolenoid::kForward);
			myShootTimer.Start();
			isShootLoop = true;
		} else if (fltShootTimer > 0.1) {
			myPiston1.Set(DoubleSolenoid::kReverse);
			myPiston2.Set(DoubleSolenoid::kReverse);
			myPiston3.Set(DoubleSolenoid::kReverse);
			myPiston4.Set(DoubleSolenoid::kReverse);
			myShootTimer.Stop();
			myShootTimer.Reset();
			intShootingMode = 0;
		} else if(trigger == false && isShootLoop == true) {
			isShootLoop = false;
		}
	}
}

void RobotCode2014::ArmControl()
{

		// If the second button on the joystick is pressed, the arm wheels rotate forward
		// If the 12th button on the joystick is pressed, the arm wheels rotate in reverse

	if(isDriveB8)
	{
		myWheelTalon.Set(1.0);
	} else if(isDriveB2 == true || intArmPosition == 2) {
		myWheelTalon.Set(-1.0);
		/*
 		if(myArmTimer.Get() == 0.0)
		{
			myWheelTalon.Set(-1.0);
			myArmTimer.Start();
		} else if(0.3 < myArmTimer.Get() && myArmTimer.Get() < 0.34) {
			myWheelTalon.Set(0.0);
		} else if(0.34 < myArmTimer.Get()) {
			myArmTimer.Stop();
			myArmTimer.Reset();
		}
		*/
	} else {
		myWheelTalon.Set(0.0);
	}
		// If joystentick buttons 7, 8, or 9 are pressed, arm position changes
		// Not implemed yet

	if (isDriveB5 == true || isDriveB3 == true)
	{
		intArmPosition = 0;
	} else if(isDriveB11) {
		intArmPosition = 1;
	} else if(isDriveB9) {
		intArmPosition = 2;
	} else if(isDriveB7){
		intArmPosition = 3;
	}


	if(intArmPosition > 0 && myArmPID.IsEnabled() == false)
	{
		myArmPID.Enable();
	} else if(intArmPosition == 0 && myArmPID.IsEnabled() == true) {
		myArmPID.Disable();
	}

	if (intArmPosition == 0)
	{
		if(isDriveB5)
		{
			myArmTalon.Set(0.6);
		} else if(isDriveB3) {
			myArmTalon.Set(-0.6);
		} else {
			myArmTalon.Set(0.0);
		}
	} else if(intArmPosition == 1) {
		myArmPID.SetSetpoint(ConvertToAngle(15.0));
	} else if(intArmPosition == 2) {
		myArmPID.SetSetpoint(ConvertToAngle(90.0));
	} else if (intArmPosition == 3) {
		myArmPID.SetSetpoint(ConvertToAngle(120.0));
	}
}

float RobotCode2014::ConvertToAngle(float fltAnalogInput)
{
	fltAnalogInput = (fltAnalogInput * (320.0 / 90.0)) + 530.0;
	//960-580
	return fltAnalogInput;
}
