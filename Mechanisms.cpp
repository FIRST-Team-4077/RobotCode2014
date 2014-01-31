#include "RobotCode2014.h"

void RobotCode2014::Shoot(bool trigger)
{
	if(trigger == true && myPistonClosed.Get() == true)
	{
		//Set pistons to forward if trigger is pulled and piston is closed
		myPiston1.Set(DoubleSolenoid::kForward);
		myPiston2.Set(DoubleSolenoid::kForward);
		myPiston3.Set(DoubleSolenoid::kForward);
		myPiston4.Set(DoubleSolenoid::kForward);
	}
	else if (myPistonOpen.Get())
	{
		//Set pistons to reverse when the piston is open
		myPiston1.Set(DoubleSolenoid::kReverse);
		myPiston2.Set(DoubleSolenoid::kReverse);
		myPiston3.Set(DoubleSolenoid::kReverse);
		myPiston4.Set(DoubleSolenoid::kReverse);
	}
}

void RobotCode2014::ArmControl()
{
	
}
