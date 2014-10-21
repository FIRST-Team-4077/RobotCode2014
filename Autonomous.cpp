#include "RobotCode2014.h"

void RobotCode2014::Autonomous(void)
{
	bool firstTimein = true;
	bool shootFirstHalf = false;
	bool shotTaken = false;
	float x = 0.0;
	fltShootTimer = 0.0;

	isShootLoop = false;
	isAutonomous = true;

	//Disable watchdog - user motor safety timer

	GetWatchdog().SetEnabled(false);	

	//Set the timeout period for the motors.
	//myTimer.Reset();

	//myEWHSRobot.SetExpiration(MOTOR_EXPIRATION_TIMEOUT_OPERATOR_CONTROL); 
	//myEWHSRobot.SetSafetyEnabled(false); //Enable the motor safety feature
	
	myDriverStationLCD->Clear();
	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line1, "Autonomous");
	myDriverStationLCD->UpdateLCD();
	myCameraServo.SetAngle(110.0);
	
	while (IsAutonomous() && IsEnabled())
	{
		if (false)
		{
			/*if (firstTimein == true)
			{
				myDriveSystem.DirectDrive(0.0, 0.0, 0.0, 0.0);
				//if (!myArmPID.IsEnabled()) myArmPID.Enable();
				myWheelTalon.Set(-0.5);
				//myArmPID.SetSetpoint(ConvertToAngle(118.0));
				myArmTalon.Set(0.3);
				// If compressor is off, turns it on
				if (myCompressor.Enabled() == false) myCompressor.Start();

				visionProcessing(); //Perform Vision Processing
				foundHotTarget = false;
				Wait(0.10);
				visionProcessing(); // Use second attempt
				if (foundHotTarget) //if foundhottarget then shoot in first 5 seconds.
				{
					myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Hot ");
					myDriverStationLCD->UpdateLCD();
					shootFirstHalf = true;
				}
				else //no targetdetected on side we are on - wait to shoot after 5 seconds
				{
					myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line2, "No Target");
					myDriverStationLCD->UpdateLCD();
					shootFirstHalf = false;
				}
				firstTimein = false;
			}

			if (shotTaken == false && shootFirstHalf == true) 
			{
				myAutonomousTimer.Start();
				x = 0.0;
				while (x < 0.9)
				{
					x = myAutonomousTimer.Get();
					myDriveSystem.DirectDrive(x, x, -x, -x);
				}
				myAutonomousTimer.Stop();
				myAutonomousTimer.Reset();
				Wait(0.25);
				myWheelTalon.Set(0.5);
				Wait(0.7);
				Shoot(true);
				myDriveSystem.DirectDrive(0.0, 0.0, 0.0, 0.0);
				LCDUpdate();
				shotTaken = true;
			} else if (shotTaken == false && shootFirstHalf == false) {
				Wait(5.0);
				myAutonomousTimer.Start();
				x = 0.0;
				while (x < 0.9)
				{
					x = myAutonomousTimer.Get();
					myDriveSystem.DirectDrive(x, x, -x, -x);
				}
				myAutonomousTimer.Stop();
				myAutonomousTimer.Reset();
				Wait(0.25);
				myWheelTalon.Set(0.5);
				Wait(0.7);
				Shoot(true);
				myDriveSystem.DirectDrive(0.0, 0.0, 0.0, 0.0);
				LCDUpdate();
				shotTaken = true;
			}*/
		} else {
			if(firstTimein == true)
			{
				firstTimein = false;
				if (!myArmPID.IsEnabled()) myArmPID.Enable();
				myArmPID.SetSetpoint(ConvertToAngle(15.0));
				myAutonomousTimer.Start();
				x = 0.0;
				while (x < 0.9)
				{
					x = myAutonomousTimer.Get();
					myDriveSystem.DirectDrive(x, x, -x, -x);
				}
				myAutonomousTimer.Stop();
				myAutonomousTimer.Reset();
				Wait(0.8);
				myDriveSystem.DirectDrive(0.0, 0.0, 0.0, 0.0);
			}
			/*
			 * 5300 rpm
			 * 88 rps
			 * 7 rps output
			 * 25 fps MAX
			 * 20 fps estimate max
			 * 18 feet red/blue zone
			
			myWheelTalon.Set(1.0);
			if (!myArmPID.IsEnabled()) myArmPID.Enable();
			myArmPID.SetSetpoint(ConvertToAngle(130.0));
			myAutonomousTimer.Start();
			x = 0.0;
			while (x < 0.25)
			{
				x = myAutonomousTimer.Get();
				myDriveSystem.DirectDrive(-x, -x, x, x);
			}
			myAutonomousTimer.Stop();
			myAutonomousTimer.Reset();
			myAutonomousTimer.Start();
			x = 0.0;
			while (x < 0.25)
			{
				x = myAutonomousTimer.Get();
				myDriveSystem.DirectDrive(x, x, -x, -x);
			}
			myAutonomousTimer.Stop();
			myAutonomousTimer.Reset();
			Wait(3.0);
			Shoot(true);
			Wait(0.1);
			myArmPID.SetSetpoint(ConvertToAngle(88.0));
			myWheelTalon.Set(1.0);
			Shoot(false);
			Wait(0.6);
			myArmPID.SetSetpoint(ConvertToAngle(130.0));
			myAutonomousTimer.Start();
			x = 0.0;
			while (x < 0.25)
			{
				x = myAutonomousTimer.Get();
				myDriveSystem.DirectDrive(-x * 0.63, -x * 0.63, x, x);
			}
			Wait(0.5);
			myAutonomousTimer.Stop();
			myAutonomousTimer.Reset();
			myDriveSystem.DirectDrive(0.0, 0.0, 0.0, 0.0);
			Shoot(true);
			*/
		}
	}
	
}
	
	

