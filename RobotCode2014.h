#include "WPILib.h"
#include "Math.h"
#include "DriveSystem.h"
//#include "types/vxTypes.h"
//#include "Timer.h"
#include "Joystick.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"


class RobotCode2014 : public SimpleRobot
{
public:

	/*

				LCD Lines

	Line 1 = Null
	Line 2 = Null
	Line 3 = Null
	Line 4 = Null
	Line 5 = Null

				Drive Joystick

	X Axis = Horizontal Movement
	Y Axis = Vertical Movement
	Z Axis = Rotational Movement
	Trigger = Shoot
	Button 2 = Run arm wheels
	Button 3 = Move arm up
	Button 4 = Camera angle up
	Button 5 = Move arm down
	Button 6 = Camera angle down
	Button 7 = Set arm to shoot mode
	Button 8 = Reverse arm wheels
	Button 9 = Set arm to ball pickup mode
	Button 10 = Weak shooting
	Button 11 = Set arm to initial Position
	Button 12 = Strong shooting

	 */

			//  Define PWM Data

	static const uint32_t FRONT_LEFT_DRIVE_MOTOR_PORT = 1;
	static const uint32_t REAR_LEFT_DRIVE_MOTOR_PORT = 2;
	static const uint32_t FRONT_RIGHT_DRIVE_MOTOR_PORT = 3;
	static const uint32_t REAR_RIGHT_DRIVE_MOTOR_PORT = 4;
	static const uint32_t ARM_TALON_CHANNEL = 5;
	static const uint32_t WHEEL_TALON_CHANNEL = 6;
	static const uint32_t CAMERA_SERVO_CHANNEL = 7;
	//static const uint32_t UNDEFINED = 8;
	//static const uint32_t UNDEFINED = 9;
	//static const uint32_t UNDEFINED = 10;

			//  Define Anolog in Data

	static const uint32_t ARM_POTENTIOMETER_CHANNEL = 1;
	//static const uint32_t UNDEFINED = 2;
	//static const uint32_t UNDEFINED = 3;
	//static const uint32_t UNDEFINED = 4;
	//static const uint32_t UNDEFINED = 5;
	//static const uint32_t UNDEFINED = 6;
	//static const uint32_t UNDEFINED = 7;
	//static const uint32_t UNDEFINED = 8;

			//	Define relay ports

	static const uint32_t COMPRESSOR_RELAY = 1;
	//static const uint32_t UNDEFINED = 2;
	//static const uint32_t UNDEFINED = 3;
	//static const uint32_t UNDEFINED = 4;
	//static const uint32_t UNDEFINED = 5;
	//static const uint32_t UNDEFINED = 6;

			//	Define digital I/O ports

	static const uint32_t PNEUMATIC_PRESSURE_SWITCH = 1;
	static const uint32_t ENCODER_1_CHANNEL_A = 2;
	static const uint32_t ENCODER_4_CHANNEL_A = 3;
	static const uint32_t ENCODER_2_CHANNEL_A = 4;
	static const uint32_t ENCODER_3_CHANNEL_A = 5;
	static const uint32_t ENCODER_1_CHANNEL_B = 6;
	static const uint32_t ENCODER_4_CHANNEL_B = 7;
	static const uint32_t ENCODER_2_CHANNEL_B = 8;
	static const uint32_t ENCODER_3_CHANNEL_B = 9;
	//static const uint32_t UNDEFINED = 10;
	//static const uint32_t UNDEFINED = 11;
	//static const uint32_t UNDEFINED = 12;
	//static const uint32_t UNDEFINED = 13;
	//static const uint32_t UNDEFINED = 14;

			//	Define Solenoid Ports

	static const uint32_t PISTON1_FORWARD = 1;
	static const uint32_t PISTON1_BACKWARD = 2;
	static const uint32_t PISTON2_FORWARD = 3;
	static const uint32_t PISTON2_BACKWARD = 4;
	static const uint32_t PISTON3_FORWARD = 5;
	static const uint32_t PISTON3_BACKWARD = 6;
	static const uint32_t PISTON4_FORWARD = 7;
	static const uint32_t PISTON4_BACKWARD = 8;

	static const uint32_t DRIVE_JOYSTICK_PORT = 1;

			//  PID Variables

	static const float fltArmP = 0.01;
	static const float fltArmI = 0.0;
	static const float fltArmD = 0.001;

	//static const float fltArmP = -0.02;
	//static const float fltArmI = 0.2;
	//static const float fltArmD = 0.0;

			//  Defines when to stop motor if no commands received

	static const float MOTOR_EXPIRATION_TIMEOUT_AUTONOMOUS = 10.0;
	static const float MOTOR_EXPIRATION_TIMEOUT_OPERATOR_CONTROL = 10.0;

	//Initialize VisionSystem
	//Camera constants used for distance calculation
	#define Y_IMAGE_RES 480	 //X Image resolution in pixels, should be 120, 240, or 480
	#define VIEW_ANGLE 37.4	 //Axis M1011 camera
	#define PI 3.141592653
	//Score limits used for target indentification
	#define RECTANGULARITY_LIMIT 40
	#define ASPECT_RATIO_LIMIT 55
	//Score limits used for hot targer determination
	#define TAPE_WIDTH_LIMIT 50
	#define VERTICAL_SCORE_LIMIT 50
	#define LR_SCORE_LIMIT 50
	//Minimum aroa of particles to be considered
	#define AREA_MINIMUM 150
	//Maximum number fo particles to process
	#define MAX_PARTICLES 8

	struct Scores
	{
		double rectangularity;
		double aspectRatioVertical;
		double aspectRatioHorizontal;
	};

	struct TargetReport
	{
		int verticalIndex;
		int horizontalIndex;
		bool Hot;
		double totalScore;
		double leftScore;
		double rightScore;
		double tapeWidthScore;
		double verticalScore;
	};
	
	Scores 						*myScores;
	TargetReport 				myTarget;


	//set up vision detection
	int verticalTargets[MAX_PARTICLES];
	int horizontalTargets[MAX_PARTICLES];
	int verticalTargetCount, horizontalTargetCount;
	bool foundHotTarget;
	
	int intArmPosition, intShootingMode;

	float fltDriveXAxis, fltDriveYAxis, fltDriveZAxis, fltDriveTwistAxis, fltExponent, fltDeadBand, fltNormalizedDriveZAxis, fltArmAngle, fltShootTimer;

	bool isDriveTrigger, isDriveB2, isDriveB3, isDriveB4, isDriveB5, isDriveB6, isDriveB7, isDriveB8, isDriveB9, isDriveB10, isDriveB11, isDriveB12, isArmLoop, isShootLoop, isShootLockout, isAutonomous;

	const char* compressorState;

			//  Define Class Objects

	AnalogChannel				myArmPotentiometer;
	Compressor					myCompressor;
	DoubleSolenoid				myPiston1, myPiston2, myPiston3, myPiston4;
	Encoder						myLeftFrontDriveEncoder, myLeftRearDriveEncoder, myRightFrontDriveEncoder, myRightRearDriveEncoder;
	Joystick					myDriveJoystick;
	PIDController				myArmPID;
	Servo						myCameraServo;
	Talon						myArmTalon, myWheelTalon, myLeftFrontDriveTalon, myLeftRearDriveTalon, myRightFrontDriveTalon, myRightRearDriveTalon;
	Timer						myAutonomousTimer, myArmTimer, myShootTimer;
	DriveSystem					myDriveSystem;
	Threshold 					myThreshold;
	AxisCamera	 				&myAxisCamera;
	DriverStationLCD			*myDriverStationLCD;


	/*	//in VisionSystem (to be added)
	double computeDistance (BinaryImage *image, ParticleAnaylsisReport *report);
	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool vertical);
	bool scoreCompare(Scores scores, bool veritcal);
	double scoreRectangularity (ParticleAnalysisReport *report);
	double ratioToScore(double ratio);
	bool hotOrNot(TargetReport myTarget);
	void LCDUpdate();
	void CheckUltrasonicRange();
	void CameraAngle();
	void visionProcessing(bool  tgtDetected);*/
	
	RobotCode2014();

	void AdjustZAxisInput(); //in Electronics

	void Autonomous(); //in Autonomous

	void ArmControl(); //in Mechanisms

	double computeDistance (BinaryImage *image, ParticleAnalysisReport *report); //in VisionSystem 

	float ConvertToAngle(float fltAnalogInput); //in Mechanisms

	bool hotOrNot (TargetReport myTarget); //in VisionSystem

	void LCDUpdate(); //in Electronics

	float Limit(float fltNewValue); //in Electronics

	void OperatorControl(); //in OperatorControl

	void PollSensors();	//in PollSensors

	double ratioToScore(double ratio); //in VisionSystem

	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool vertical); //in VisionSystem

	bool scoreCompare(Scores scores, bool vertical); //in VisionSystem

	double scoreRectangularity(ParticleAnalysisReport *report); //in VisionSystem

	void Shoot(bool fire); //in Mechanisms
	
	void visionProcessing(); //in VisionSystem 

};
