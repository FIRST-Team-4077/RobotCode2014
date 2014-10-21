#include "RobotCode2014.h"

void RobotCode2014::visionProcessing() 
{
	//Particle filter criteria, used to filter out small particles
	ParticleFilterCriteria2 myCriteria[] = 
	{
			{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
	};
	
	 /*
	 * Do the image capture with the camera and apply the algorithm described above. This
	 * sample will either get images from the camera or from an image file stored 
	 * in the top level directory in the flash memory on the cRIO. 
	 * The file name in this case is "testImage.jpg"
	 */
	ColorImage *image;
	// get the sample image from the cRIO flash
	//image = new RGBImage("/testImage.jpg");		
	//To get the images from the camera comment the line above and uncomment this one
	image = myAxisCamera.GetImage();				
	// get just the green target pixels
	BinaryImage *thresholdImage = image->ThresholdHSV(myThreshold);	
	thresholdImage->Write("/threshold.bmp");

	//Remove small particles
	BinaryImage *filteredImage = thresholdImage->ParticleFilter(myCriteria, 1);	
	//filteredImage->Write("/filtered.bmp");

	
	//get a particle analysis report for each particle
	vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  

	verticalTargetCount = 0;
	horizontalTargetCount = 0;

	myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line1, "reports size: %d", reports->size());
	myDriverStationLCD->UpdateLCD();
	//Iterate through each particle, scoring it and determining whether it is a target or not
	if(reports->size() > 0)
	{
		myScores = new Scores[reports->size()];
		for (unsigned int i = 0; i < MAX_PARTICLES && i < reports->size(); i++) 
		{
			ParticleAnalysisReport *report = &(reports->at(i));
		
			//Score each particle on rectangularity and aspect ratio
			myScores[i].rectangularity = scoreRectangularity(report);
			myScores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, true);
			myScores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, false);			
		
			//Check if Horizontal target
			if(scoreCompare(myScores[i], false))
			{
				myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line2, "%d HorTgt X:%d  Y:%d", i, report->center_mass_x, report->center_mass_y);
				myDriverStationLCD->UpdateLCD();
				horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
			} //Check if vertical target
			else if (scoreCompare(myScores[i], true)) {
				myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line2, "%d VerTgt X:%d  Y:%d", i, report->center_mass_x, report->center_mass_y);
				myDriverStationLCD->UpdateLCD();
				verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
			} //Neither Horizonal or Vertical - no target
			else 
			{
				myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line2, "%d notTgt X:%d Y:%d \n", i, report->center_mass_x, report->center_mass_y);
				myDriverStationLCD->UpdateLCD();
			}
			myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line3, "Scores rect: %f", myScores[i].rectangularity);
			myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line4, "ARvert: %f", myScores[i].aspectRatioVertical);
			myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line5, "ARhoriz: %f", myScores[i].aspectRatioHorizontal);
			myDriverStationLCD->UpdateLCD();
			//Wait(10.5);
		}
		//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
		myTarget.totalScore = myTarget.leftScore = myTarget.rightScore = myTarget.tapeWidthScore = myTarget.verticalScore = 0;
		myTarget.verticalIndex = verticalTargets[0];
		for (int i = 0; i < verticalTargetCount; i++)
		{
			ParticleAnalysisReport *verticalReport = &(reports->at(verticalTargets[i]));

			for (int j = 0; j < horizontalTargetCount; j++)
			{
				ParticleAnalysisReport *horizontalReport = &(reports->at(horizontalTargets[j]));
				double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;

				//Measure equivalent rectangle sides for use in score calculation
				imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &horizWidth);
				imaqMeasureParticle(filteredImage->GetImaqImage(), verticalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &vertWidth);
				imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &horizHeight);
			
				//Determine if the horizontal target is in the expected location to the left of the vertical target
				leftScore = ratioToScore(1.2*(verticalReport->boundingRect.left - horizontalReport->center_mass_x)/horizWidth);
				//Determine if the horizontal target is in the expected location to the right of the  vertical target
				rightScore = ratioToScore(1.2*(horizontalReport->center_mass_x - verticalReport->boundingRect.left - verticalReport->boundingRect.width)/horizWidth);
				//Determine if the width of the tape on the two targets appears to be the same
				tapeWidthScore = ratioToScore(vertWidth/horizHeight);
				//Determine if the vertical location of the horizontal target appears to be correct
				verticalScore = ratioToScore(1-(verticalReport->boundingRect.top - horizontalReport->center_mass_y)/(4*horizHeight));
				total = leftScore > rightScore ? leftScore:rightScore; //if leftScore > rightScore, return leftScore else return rightScore
				total += tapeWidthScore + verticalScore;
			
				//If the target is the best detected so far store the information about it
				if(total > myTarget.totalScore)
				{
					myTarget.horizontalIndex = horizontalTargets[j];
					myTarget.verticalIndex = verticalTargets[i];
					myTarget.totalScore = total;
					myTarget.leftScore = leftScore;
					myTarget.rightScore = rightScore;
					myTarget.tapeWidthScore = tapeWidthScore;
					myTarget.verticalScore = verticalScore;
				}
			}
			//Determine if the best target is a Hot target
			myTarget.Hot = hotOrNot(myTarget);
		}

		if(verticalTargetCount > 0)
		{
			//Information about the target is contained in the "target" structure
			//To get measurement information such as sizes or locations use the
			//horizontal or vertical index to get the particle report as shown below
			ParticleAnalysisReport *distanceReport = &(reports->at(myTarget.verticalIndex));
			//double distance = computeDistance(filteredImage, distanceReport);
			if (myTarget.Hot)
			{
				myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line6, "Hot target");
				myDriverStationLCD->UpdateLCD();
				foundHotTarget = true;
			} 
			else 
			{
				myDriverStationLCD->PrintfLine(DriverStationLCD::kUser_Line6, "No hot target present");
				myDriverStationLCD->UpdateLCD();
				foundHotTarget = false;
			}
		}
		delete myScores;
	}
	
	// be sure to delete images and objects after using them
	delete filteredImage;
	delete thresholdImage;
	delete image;
	delete reports;
}

double RobotCode2014::computeDistance (BinaryImage *image, ParticleAnalysisReport *report) 
/**
 * Computes the estimated distance to a target using the height of the particle in the image. 
 * For more information and graphics showing the math behind this approach see the Vision Processing 
 * section of the ScreenStepsLive documentation.
 * 
 * @param image The image to use for measuring the particle estimated rectangle
 * @param report The Particle Analysis Report for the particle
 * @return The estimated distance to the target in feet.
 */
{
	double rectLong, height;
	int targetHeight;
	
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
	//using the smaller of the estimated rectangle long side and the bounding rectangle height 
	//results in better performance on skewed rectangles
	height = min(report->boundingRect.height, rectLong);
	targetHeight = 32;

	return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
}

double RobotCode2014::scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool vertical)
/**
 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. 
 * This method uses the equivalent rectangle sides to determine aspect ratio as it performs better 
 * as the target gets skewed by moving to the left or right. The equivalent rectangle is the rectangle 
 * with sides x and y where particle area= x*y and particle perimeter= 2x+2y
 * 
 * @param image The image containing the particle to score, needed to perform additional measurements
 * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the 
 * inner target or the outer @return The aspect ratio score (0-100)
 */
{
	double rectLong, rectShort, idealAspectRatio, aspectRatio;
	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall
	idealAspectRatio = vertical ? (4.0/32) : (23.5/4);	
	
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
	imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
	
	//Divide width by height to measure aspect ratio
	if(report->boundingRect.width > report->boundingRect.height)
	{
		//particle is wider than it is tall, divide long by short
		aspectRatio = ratioToScore(((rectLong/rectShort)/idealAspectRatio));
	} else 
	{
		//particle is taller than it is wide, divide short by long
		aspectRatio = ratioToScore(((rectShort/rectLong)/idealAspectRatio));
	}
	return aspectRatio;		//force to be in range 0-100
}

bool RobotCode2014::scoreCompare(Scores myScores, bool vertical)
/**
 * Compares scores to defined limits and returns true if the particle appears to be a target
 * 
 * @param scores The structure containing the scores to compare
 * @param vertical True if the particle should be treated as a vertical target, 
 *                 False to treat it as a horizontal target
 * 
 * @return True if the particle meets all limits, false otherwise
 */
{
	bool isTarget = true;

	isTarget &= myScores.rectangularity > RECTANGULARITY_LIMIT;
	if(vertical)
	{
		isTarget &= myScores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
	} else 
	{
		isTarget &= myScores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
	}

	return isTarget;
}

double RobotCode2014::scoreRectangularity(ParticleAnalysisReport *report)
/**
 * Computes a score (0-100) estimating how rectangular the particle is by comparing 
 * the area of the particle to the area of the bounding box surrounding it. A perfect 
 * rectangle would cover the entire bounding box.
 * 
 * @param report The Particle Analysis Report for the particle to score
 * @return The rectangularity score (0-100)
 */
{
	if(report->boundingRect.width*report->boundingRect.height !=0)
	{
		return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
	} 
	else 
	{
		return 0;
	}	
}	

double RobotCode2014::ratioToScore(double ratio)
/**
 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
 */
{
	return (max(0, min(100*(1-fabs(1-ratio)), 100)));
}

bool RobotCode2014::hotOrNot(TargetReport target)
/**
 * Takes in a report on a target and compares the scores to the defined score limits 
 * to evaluate if the target is a hot target or not and if the target is the LEFT TARGET.
 * 
 * Returns True if the target is hot. False if it is not.
 */
{
	bool isHot = true;
	isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
	isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
	isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);

	return isHot;
}

