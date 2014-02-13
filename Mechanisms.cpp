#include "RobotCode2014.h"

void RobotCode2014::Shoot(bool trigger)
{
	if(trigger == true && fltShootTimer == 0.0)
	{
			//Set pistons to forward if trigger is pulled and piston is closed
		myPiston1.Set(DoubleSolenoid::kForward);
		myPiston2.Set(DoubleSolenoid::kForward);
		myPiston3.Set(DoubleSolenoid::kForward);
		myPiston4.Set(DoubleSolenoid::kForward);
		myShootTimer.Start();
	}
	else if (fltShootTimer > 0.5)
	{
			//Set pistons to reverse when the piston is open
		myPiston1.Set(DoubleSolenoid::kReverse);
		myPiston2.Set(DoubleSolenoid::kReverse);
		myPiston3.Set(DoubleSolenoid::kReverse);
		myPiston4.Set(DoubleSolenoid::kReverse);
		myShootTimer.Stop();
		myShootTimer.Reset();
	}
}

void RobotCode2014::ArmControl()
{

		// If the second button on the joystick is pressed, the arm wheels rotate forward

	if(isDriveB2)
	{
		myWheelTalon.Set(-1.0);
	} else if(isDriveB4) {
		myWheelTalon.Set(1.0);
	} else {
		myWheelTalon.Set(0.0);
	}

		// If the fifth or sixth button on the joystick is pressed, the arm moves up or down

	

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

	if(intArmPosition > 0 && myArmPID.IsEnabled() == false) myArmPID.Enable();
	if (intArmPosition == 0)
	{
		if(isDriveB5)
		{
			myArmTalon.Set(0.5);
		} else if(isDriveB3) {
			myArmTalon.Set(-0.5);
		} else {
			myArmTalon.Set(0.0);
		}
	} else if(intArmPosition == 1) {
		myArmPID.SetSetpoint(ConvertToAngle(0.0));
	} else if(intArmPosition == 2) {
		myArmPID.SetSetpoint(ConvertToAngle(100.0));
	} else if (intArmPosition == 3) {
		myArmPID.SetSetpoint(ConvertToAngle(125.0));
	} else {
		myArmPID.Disable();
	}
}

float RobotCode2014::ConvertToAngle(float fltAnalogInput)
{
	fltAnalogInput = -(fltAnalogInput * (320.0 / 90.0)) + 830.0;
	return fltAnalogInput;
}
