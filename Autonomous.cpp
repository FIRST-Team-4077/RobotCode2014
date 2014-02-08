#include "RobotCode2014.h"

void RobotCode2014::Autonomous()
{
	while(IsAutonomous() && IsEnabled())
	{
			// If compressor is off, turns it on

		if (myCompressor.Enabled() == false) myCompressor.Start();

			// This timer allows the Autonomous code to do certain actions at specific times

		if (myAutonomousTimer.Get() == 0) myAutonomousTimer.Start();
		if(myAutonomousTimer.Get() < 3.0)
		{
			myDriveSystem.DriveDistance(5.0);
		} else if(3.0 < myAutonomousTimer.Get() && myAutonomousTimer.Get() < 15.0) {
			// Vision processing code goes here, and so does shooting code
			//Idea(brandon): if (goal.see = true && distance = ___){}
			Shoot(true);
		}
	}
}
