#include "WPILib.h"

#include <math.h>

#include "Vision/AxisCamera.h"
#include "Vision/HSLImage.h"
#include "Target.h"
#include "PIDSpin.h"
#include "PIDCamera.h"
#include "DriverStation.h"
#include "DashboardDataFormat.h"
#include "CANJaguar/CANJaguar.h"
#include "PIDDrive.h"
#include "PIDCameraZoom.h"
#include "DriverStationEnhancedIO.h"
#include "CameraTarget.h"
#include "AverageEncoder.h"
#include "EncoderSource.h"
#include "DualSpeedController.h"
#include "WinchOutput.h"


 //Precondition: Kicker must be ready to fire, winched out and armed 

class Robot2010 : public SimpleRobot
{

//Motors and drive system
	Joystick lstick, rstick, stick3; // only joystick
	CANJaguar leftFrontMotor;
	CANJaguar leftRearMotor;
	CANJaguar rightFrontMotor;
	CANJaguar rightRearMotor;
	CANJaguar kickerMotor1;
	CANJaguar kickerMotor2;
	DualSpeedController kickerMotor;
	CANJaguar intakeMotor;
	Servo cameraServo; 
	RobotDrive myRobot; // robot drive system
//Pneumatics
	Solenoid shifter1;
	Solenoid shifter2;
	Solenoid ratchet1;
	Solenoid ratchet2;
	Compressor* robotCompressor;
//Camera
	PIDSpin spin;
	PIDController spinControl;
	PIDSpin aim;
	PIDCamera aimCamera;
	PIDController aimControl;
	PIDDrive zoomDrive;
	PIDCameraZoom zoomCamera;
	PIDController zoomControl;
//Sensors
	Gyro gyro;
	DigitalInput frontLimitSwitch;
	Encoder kickerEncoder; //comment out
	Encoder leftDriveEncoder;
	Encoder rightDriveEncoder;
//	Accelerometer XAccelerometer;
//	Accelerometer YAccelerometer;
//	HiTechnicCompass compass; */
	DigitalInput backLimitSwitch;
//	DigitalInput cameraLimitSwitch;
//	int sign;
	
	AverageEncoder avgEnc;
	EncoderSource leftEnc;
	PIDDrive autoDrive;
	PIDController autoController;

	EncoderSource winchEnc;
	WinchOutput winchOutput;
	PIDController winchControl;
	
	int autoMode;
	const double gDriveTicksPerInch;

	Timer kickTime;
	Timer kickPneumaticTime;

	// encoder value for fully winched out and ready to kick
	const int gWinchOut;
	// encoder value for fully winched back (full power kick)
	const int gWinchFull;
	// deadband on encoder controller
	const int gWinchDeadband;
	// how long to wait after kick pneumatic release to start winching back again
	const double gWinchWaitTime;
	// how long to wait after kick pneumatic release until we can kick again (rule: 2 seconds between kicks)
	const double gBetweenKickTime;
	// remembers current retract setting, near, mid, far


	
	int currentSetting;
	bool armed;
	
	//New code for adjustable stuff starts here
	
	int midRetract;
	int farRetract;
	int nearRetract;
	

public:
	Robot2010(void):
	//Motors and drive system
		lstick(1),		// as they are declared above.
		rstick(2),
		stick3(3),
		leftFrontMotor(2),
		leftRearMotor(7),
		rightFrontMotor(4),
		rightRearMotor(6),
		kickerMotor1(5),
		kickerMotor2(8),
		kickerMotor(kickerMotor1, kickerMotor2),
		intakeMotor(3),
		cameraServo(1),
		myRobot(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor),	// these must be initialized in the same order
	//Pneumatics
		shifter1(7,1),
		shifter2(7,2),
		ratchet1(7,3),
		ratchet2(7,4),
		spin(myRobot),
		spinControl(1.0/90,0,0,&gyro,&spin),
		aim(myRobot),
		aimControl(1.0/20,0,0,&gyro,&aim),		
		zoomDrive(myRobot),
		zoomCamera(&zoomControl),
		zoomControl(1.0/36.0,0,0,&zoomCamera,&zoomDrive,.1),
	//Sensors
		gyro(1),
		frontLimitSwitch(6),
		kickerEncoder(9,10,true),
		leftDriveEncoder(13,14,true),
		rightDriveEncoder(11,12),
	//	XAccelerometer(2),
	//	YAccelerometer(3)
		// compass(4) */
		backLimitSwitch(5),
	//	cameraLimitSwitch(7),
		
		
		avgEnc(leftDriveEncoder, rightDriveEncoder),
		leftEnc(leftDriveEncoder),
		autoDrive(myRobot),
		autoController(-0.016, 0.0, 0.0, &leftEnc, &autoDrive),

		winchEnc(kickerEncoder),
		winchOutput(kickerMotor, kickerEncoder, backLimitSwitch),
		winchControl(-1.0/25.0, 0.0, -1.0/25.0, &winchEnc, &winchOutput),
		
		autoMode(1),
		gDriveTicksPerInch(11),
		gWinchOut(570),
		gWinchFull(290),
		gWinchDeadband(8),
		gWinchWaitTime(0.25),
		gBetweenKickTime(2.375),
		armed(false)
	{
		GetWatchdog().SetExpiration(0.25);
		gyro.SetSensitivity(0.007);
		//myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		//myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		//myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		//myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	}

	void RobotInit(void)
	{
		Wait(0.25);  //could turn to be a  lot less
		AxisCamera& camera = AxisCamera::GetInstance();
		//CameraTarget& target = CameraTarget::getInstance();
		spinControl.SetOutputRange(-0.6,0.6);
		aimControl.SetOutputRange(-0.6,0.6);
		zoomControl.SetOutputRange(-0.4,0.4);
		//winchControl.SetInputRange(-gWinchOut, gWinchFull);
		winchControl.SetOutputRange(-1.0,1.0);
		kickerEncoder.Start();
		leftDriveEncoder.Start();
		rightDriveEncoder.Start();
		robotCompressor = new Compressor(8,1);
		robotCompressor->Start();
		
		kickTime.Start();
		kickPneumaticTime.Start();
	}
	
	void DisplayAuto(void)
	{
		DriverStationLCD& dselcd = *DriverStationLCD::GetInstance();
		switch (autoMode)
		{
		case 0:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A0: do nothing");
			printf("Autonomous do nothing\n");
			break;
		case 1:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A1: far 8ft 0s");
			printf("Autonomous kick everything for 8 ft in far zone, no delay\n");
			break;
		case 2:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A2: far 8ft 1s");
			printf("Autonomous kick everything for 8 ft in far zone, 1 second delay\n");
			break;
		case 3:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A3: far 8ft 2s");
			printf("Autonomous kick everything for 8 ft in far zone, 2 second delay\n");
			break;
		case 4:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A4: mid 8ft 0s");
			printf("Autonomous kick everything for 8 ft in mid zone, no delay\n");
			break;
		case 5:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A5: mid 8ft 1s");
			printf("Autonomous kick everything for 8 ft in mid zone, 1 second delay\n");
			break;	
		case 6:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A6: mid 8ft 2s");
			printf("Autonomous kick everything for 8 ft in mid zone, 2 second delay\n");
			break;	
		case 7:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A7: near 8ft 0s");
			printf("Autonomous near zone\n");
			break;	
		case 8:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A8: far 0s over");
			printf("Autonomous kick everything for 8 ft in far zone, no delay, drive over bump\n");
			break;
		case 9:
			dselcd.PrintfLine(DriverStationLCD::kUser_Line1, "A9: far 0s over bounce");
			printf("Autonomous kick everything for 8 ft in far zone, no delay,with bounce\n");
			break;
		}
		dselcd.UpdateLCD();
	}

	void Disabled(void)
	{
		bool rPrevious = false;
		bool lPrevious = false;
		
		DriverStationEnhancedIO &dseio = DriverStation::GetInstance()->GetEnhancedIO();
		DisplayAuto();

		//Chooses the autonomous mode, right trigger increments autoMode and left trigger decrements autoMode
		//autoMode is displayed in binary on the LEDs on the Cypress board
		while (IsDisabled())
		{
			bool display = false;
			if(lstick.GetTrigger() && !lPrevious)
			{
				autoMode--;
				display = true;
			}
			else if(rstick.GetTrigger() && !rPrevious)
			{
				autoMode++;
				display = true;
			}
			if(autoMode < 0)
			{
				autoMode = 0;
				display = false;
			}
			else if (autoMode > 9)
			{
				autoMode = 9;
				display = false;
			}
//			dseio.SetLEDs(autoMode);
			if (display)
				DisplayAuto();
			
			rPrevious = rstick.GetTrigger();
			lPrevious = lstick.GetTrigger();
	/*		if (lstick.GetTrigger())
			{
				autoMode = 1;
				dseio.SetDigitalOutput(1,1);
				dseio.SetDigitalOutput(2,0);
				dseio.SetDigitalOutput(3,0);
			}
			if (rstick.GetTrigger()) 
			{
				autoMode = 2;
				dseio.SetDigitalOutput(1,0);
				dseio.SetDigitalOutput(2,1);
				dseio.SetDigitalOutput(3,0);
			}
			if (stick3.GetTrigger()) 
			{
				autoMode = 3;
				dseio.SetDigitalOutput(1,0);
				dseio.SetDigitalOutput(2,0);
				dseio.SetDigitalOutput(3,1);
			}*/
			Wait(0.01);
		}
	}

	bool HaveBall()
	{
		return (intakeMotor.GetOutputCurrent() > 10); // was 15
	}
	
	// winch control of kicker (used in both auto and teleop)
	// returns true when winch target reached
	bool RunWinch(int winchTarget)
	{
#if 0
		int error = kickerEncoder.Get() - winchTarget;

		// use simple binary control of the winch with deadband
		// invert the kicker encoder value due to direction
		if (error < -gWinchDeadband)
		{
			if (backLimitSwitch.Get() == 0)
			{
				kickerMotor.Set(0.0);
				return true;
			}
			else
			{
				if (error < -30)
					kickerMotor.Set(-1.0);
				else
					kickerMotor.Set(error/30.0);
			}
		}
		else if (error > gWinchDeadband)
		{
			if (error > 30)
				kickerMotor.Set(1.0);
			else
				kickerMotor.Set(error/30.0);
		}
		else
		{
			kickerMotor.Set(0.0);
			return true;
		}
		return false;
#else
		if (IsWinchOnTarget(winchTarget))
		{
			winchControl.Disable();
			return true;
		}
		// Use PID control
		winchControl.Enable();
		winchControl.SetSetpoint(winchTarget);
		if (armed)
			winchControl.SetPID(-1.0/50.0, 0.0, -1.0/50.0);
		else
			winchControl.SetPID(-1.0/30.0, 0.0, 0.0);
		return IsWinchOnTarget(winchTarget);
#endif
	}
	
	bool IsWinchOnTarget(int winchTarget)
	{
		if (winchTarget >= gWinchFull && backLimitSwitch.Get() == 0)
			return true;
		int error = kickerEncoder.Get() - winchTarget;
		if (error < -gWinchDeadband)
			return false;
		else if (error > gWinchDeadband)
			return false;
		else
			return true;
	}

	void ArmKicker(void)
	{
		ratchet1.Set(true);
		ratchet2.Set(false);
		armed = true;
		kickPneumaticTime.Reset();
		printf("ARMED!!\n");
	}
	
	void FireKicker(void)
	{
		intakeMotor.Set(0.0);
		ratchet1.Set(false);
		ratchet2.Set(true);
		armed = false;
		kickPneumaticTime.Reset();
		kickTime.Reset();
		printf("KICKED!!\n");
	}

	void ClearRatchetPneumatics(void)
	{
		ratchet1.Set(false);
		ratchet2.Set(false);
	}

	void ResetDriveEncoders(void)
	{
		leftDriveEncoder.Reset();
		rightDriveEncoder.Reset();
	}

	// autonomous-only arming of kicker
	// returns true when kicker is ready to fire
	bool AutoArmKicker(int power, bool do_arm=true)
	{
		if (!armed)
		{
			// winch back
			if (RunWinch(power) && do_arm)
				ArmKicker();	// arm when we're done winching back
		}
		else
		{
			// we're armed, so unwinch
			if (RunWinch(-gWinchOut) && kickTime.Get() > gBetweenKickTime)
				return true;	// armed and ready to fire!
		}
		return false;
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		GetWatchdog().SetEnabled(false);
		//gyro.Reset();
		Timer time;
		Timer haveTime;
		Timer waitTime;
		
		ResetDriveEncoders();
		kickerEncoder.Reset(); //zeroing kicker
		
		time.Start();
		haveTime.Start();
		waitTime.Start();

		int prevstate = -1;
		int state = 0;
		int i=0;
		int kickPower = 188;
		int kickPowerFar[5] = {188 + 66, 188 +66, 152 +66, 152 +33, 152 +33}; 
		int kickPowerFarBounce[5] = {188 - 66, 188 - 99, 152 - 66, 152 - 66, 152 - 66}; 
		//int kickPowerFar[5] = {50, 50, 50, 50, 50};
		int kickPowerMid[5] = {50 + 33, 50 + 33, 50 + 33, 50 + 33, 50 + 33};
		int kickPowerNear = 0;
		float startDelay = 0.0;
		int* kickPowerUse = kickPowerFar;
		int ballsKicked = 0;
		float intakeSpeed = -1.0;

		autoController.Enable();

		// Set up kick power for each mode
		if (autoMode == 1 || autoMode == 2 || autoMode == 3 || autoMode == 8)
			kickPowerUse = kickPowerFar; // far zone kick strengths
		else if (autoMode == 4 || autoMode == 5 || autoMode == 6)
			kickPowerUse = kickPowerMid; // mid zone kick strengths
		else if (autoMode == 9)
			kickPowerUse = kickPowerFarBounce; // far zone with bounce kick strengths
		
		// Set up start delay for each mode
		if (autoMode == 1 || autoMode == 4)
			startDelay = 0.0;
		else if (autoMode == 2 || autoMode == 5)
			startDelay = 1.0;
		else if (autoMode == 3 || autoMode == 6)
			startDelay = 2.0;		
		
		while (IsAutonomous())
		{
			GetWatchdog().Feed();

			// pulse the pneumatics (100 ms minimum pulse)
			if (kickPneumaticTime.Get() > 0.3)
			{
				ClearRatchetPneumatics();
			}

			// always run the intake motor
			intakeMotor.Set(intakeSpeed);
			if (prevstate != state)
				printf("AUTO mode:%d state:%d\n", autoMode, state);
			prevstate = state;

			// ball detection
			if (!HaveBall())
				haveTime.Reset();

			i++;
#if 0
			if (autoMode == 1 or autoMode == 2 or autoMode == 3) // kick automode # of times and winch back
			{
				switch (state)
				{
				case 0:
					// Drive forward while setting up for kick
					if (i%2 == 0)
						printf("S0: moving to first ball; winch:%d, drive:%f\n", kickerEncoder.Get(), autoController.GetError());
					AutoArmKicker(kickPower);
					autoController.SetSetpoint(16*gDriveTicksPerInch); //41 inches
			        if (fabs(autoController.GetError()) < 1.5*gDriveTicksPerInch) // less than 1.5 inch of error 
			        	state = 1;
			        break;
				case 1:
					// wait until we're ready to fire, then fire!
					if (i%2 == 0)
						printf("S1: firing first ball; winch:%d\n", kickerEncoder.Get());
					if (AutoArmKicker(kickPower))
					{
						FireKicker();
						state = 2;
					}
					break;
				case 2:
					// wait until we finished kicking
					if (i%2 == 0)
						printf("S2: waiting to finish kick 1\n");
					if (kickTime.Get() > gWinchWaitTime)
					{
						if (autoMode == 1)
							state = 10;
						else
						{
							ResetDriveEncoders();
							autoController.SetSetpoint(37*gDriveTicksPerInch); //36 inches
							state = 3;
						}
					}
					break;
				case 3:
					// Drive forward while setting up for kick 2
					if (i%2 == 0)
						printf("S3: moving to second ball; winch:%d, drive:%f\n", kickerEncoder.Get(), autoController.GetError());
					AutoArmKicker(kickPower); // needs to be lower than kick one, as per testing
			        if (fabs(autoController.GetError()) < 1.5*gDriveTicksPerInch) // less than 1.5 inch of error 
			        	state = 4;
			        break;
				case 4:
					// wait until we're ready to fire, then fire!
					if (i%2 == 0)
						printf("S4: firing second ball; winch:%d\n", kickerEncoder.Get());
					if (AutoArmKicker(kickPower))
					{
						FireKicker();
						state = 5;
					}
					break;
				case 5:
					// wait until we finished kicking
					if (i%2 == 0)
						printf("S6: waiting to finish kick 2\n");
					if (kickTime.Get() > gWinchWaitTime)
					{
						if (autoMode == 3)
						{
							ResetDriveEncoders();
							autoController.SetSetpoint(37*gDriveTicksPerInch); //36 inches
							state = 6;
						}
						else
							state = 10;
					}
					break;
				case 6:
					// Drive forward while setting up for kick 3
					if (i%2 == 0)
						printf("S6: moving to third ball; winch:%d, drive:%f\n", kickerEncoder.Get(), autoController.GetError());
					AutoArmKicker(kickPower); // Should be less powerfull than kick two, need to change after testing
					if (fabs(autoController.GetError()) < 1.5*gDriveTicksPerInch) // less than 1.5 inch of error 
						state = 7;
					break;
				case 7:
					// wait until we're ready to fire, then fire!
					if (i%2 == 0)
						printf("S7: firing third ball; winch:%d\n", kickerEncoder.Get());
					if (AutoArmKicker(152))
					{
						FireKicker();
						state = 8;
					}
					break;
				case 8:
					// wait until we finished kicking
					if (i%2 == 0)
						printf("S8: waiting to finish kick 3\n");
					if (kickTime.Get() > gWinchWaitTime)
					{
						state = 10;
					}
					break;
				case 10:
					// make sure we winch back so we don't get a penalty
					if (i%2 == 0)
						printf("S10: making sure kicker is retracted\n");
					RunWinch(0);
					break;
					
				}
			}
			else
#endif
			if (autoMode >= 1 && autoMode <= 6 || autoMode == 8 || autoMode == 9) // drive forward and auto-kick
			{
				autoController.Disable();
				switch (state)
				{
				case 0:
					// Wait to start
					if (time.Get() > startDelay)
						state = 1;
					break;
				case 1:
					// Drive forward while setting up for kick
					if (i%2 == 0)
						printf("S0: moving to ball; winch:%d\n", kickerEncoder.Get());
					AutoArmKicker(kickPowerUse[ballsKicked]);
					myRobot.ArcadeDrive(-0.5, 0.0, false); // half speed drive
			        if (time.Get() > 0.25 && HaveBall())
			        {
			        	printf("GOT BALL!!\n");
			        	// got a ball; kick it!
			        	myRobot.ArcadeDrive(0.0, 0.0, false);
			        	state = 2;
			        }
			        if (leftDriveEncoder.Get() > 102*gDriveTicksPerInch)
			        	state = 4;
			        // stop after 2 balls kicked in mid zone
			        if (autoMode >= 4 && autoMode <= 6 && ballsKicked >= 2)
			        	state = 4;
			        // stop after 3 balls kicked in far zone
					if ((autoMode == 8 || autoMode == 9) && ballsKicked >= 3)
						state = 4;
			        break;
				case 2:
					// wait until we're ready to fire, then fire!
					if (i%2 == 0)
						printf("S1: firing ball; winch:%d\n", kickerEncoder.Get());
					if (AutoArmKicker(kickPowerUse[ballsKicked]))
					{
						FireKicker();
						ballsKicked++;
						if (ballsKicked > 4)
							ballsKicked = 4;
						state = 3;
					}
					break;
				case 3:
					// wait until we finished kicking
					if (i%2 == 0)
						printf("S2: waiting to finish kick\n");
					if (kickTime.Get() > gWinchWaitTime)
						state = 1;
					break;
				case 4:
					if (autoMode >= 4 && autoMode <= 6) // mid zone modes
						state = 5;
					else if (autoMode == 8 || autoMode == 9) // far zone go over bump
						state = 8;
					else
						state = 20;
					break;
				case 5:
					// back up
					AutoArmKicker(kickPowerUse[ballsKicked]);
		        	myRobot.ArcadeDrive(1.0, 0.0, false);
			        if (leftDriveEncoder.Get() < 24*gDriveTicksPerInch)
			        {
			        	waitTime.Reset();
			        	state = 6;
			        }
					break;
				case 6:
					// spin 90 deg
					AutoArmKicker(kickPowerUse[ballsKicked]);
		        	myRobot.ArcadeDrive(0.0, 0.85, false);
		        	if (waitTime.Get() > 0.85)
		        		state = 20;
					break;
				case 8:
					// dry kick before driving over bump if we're armed
					if (armed)
					{
						// finish arming and fire
						if (AutoArmKicker(kickPowerUse[ballsKicked]))
						{
							FireKicker();
							state = 9;
						}
					}
					else
						state = 10;
					break;
				case 9:
					if (kickTime.Get() > gWinchWaitTime)
						state = 10;
					break;
				case 10:
					// drive over bump
					intakeSpeed = 1.0;	// run roller out so we don't get penalties
					AutoArmKicker(kickPowerUse[ballsKicked], false);
			        if (leftDriveEncoder.Get() > (96+36)*gDriveTicksPerInch)
			        {
			        	waitTime.Reset();
			        	state = 11;
			        }
					myRobot.ArcadeDrive(-0.5, 0.0, false); // drive forward
					break;
				case 11:
					AutoArmKicker(kickPowerUse[ballsKicked], false);
					myRobot.ArcadeDrive(-0.85, 0.0, false); // drive forward higher power
		        	if (waitTime.Get() > 3.0)
		        		state = 20;
					break;
				case 20:
					// make sure we winch back so we don't get a penalty
					if (i%2 == 0)
						printf("S10: done\n");
					AutoArmKicker(kickPowerUse[ballsKicked], false);
		        	myRobot.ArcadeDrive(0.0, 0.0, false);
					break;
				}
			}
			else if (autoMode == 7) // near zone: drive forward and auto-kick 1 ball, then back up
			{
				autoController.Disable();
				switch (state)
				{
				case 0:
					// Wait to start
					if (time.Get() > startDelay)
						state = 1;
					break;
				case 1:
					// Drive forward while setting up for kick
					if (i%2 == 0)
						printf("S0: moving to ball; winch:%d\n", kickerEncoder.Get());
				//	if (time.Get() >1)
				//	{
						AutoArmKicker(kickPowerNear);
						myRobot.ArcadeDrive(-0.5, 0.0, false); // half speed drive
				//	}
			        if (time.Get() > 0.25 && haveTime.Get() > 0.25)
			        {
			        	printf("GOT BALL!!\n");
			        	// got a ball; kick it!
			        	myRobot.ArcadeDrive(0.0, 0.0, false);
			        	state = 2;
			        }
			        if (leftDriveEncoder.Get() > 6*12*gDriveTicksPerInch)
			        	state = 4;
			        break;
				case 2:
					// wait until we're ready to fire, then fire!
					if (i%2 == 0)
						printf("S1: firing ball; winch:%d\n", kickerEncoder.Get());
					if (AutoArmKicker(kickPowerNear))
					{
						FireKicker();
						state = 3;
					}
					break;
				case 3:
					// wait until we finished kicking
					if (i%2 == 0)
						printf("S2: waiting to finish kick\n");
					if (kickTime.Get() > gWinchWaitTime)
						state = 4;
					break;
				case 4:
					if (i%2 == 0)
						printf("S4: driving back\n");
					myRobot.ArcadeDrive(1.0, 0.0, false); // full speed drive backwards
			        if (leftDriveEncoder.Get() < 24*gDriveTicksPerInch)
			        {
			        	waitTime.Reset();
			        	state = 5;
			        }
					if (kickTime.Get() > gWinchWaitTime && AutoArmKicker(kickPowerNear) && haveTime.Get() > 0.25)
					{
						FireKicker();
					}
					break;
				case 5:
					if (i%2 == 0)
						printf("S5: wait to drive forward again\n");
					if (kickTime.Get() > gWinchWaitTime && AutoArmKicker(kickPowerNear) && haveTime.Get() > 0.25)
					{
						FireKicker();
					}
					if (waitTime.Get() > 5.0)
						state = 6;
					break;
				case 6:
					if (i%2 == 0)
						printf("S5: drive forward slowly kicking balls again\n");
					if (kickTime.Get() > gWinchWaitTime && AutoArmKicker(kickPowerNear) && haveTime.Get() > 0.25)
					{
						FireKicker();
					}
					myRobot.ArcadeDrive(-0.35, 0.0, false); // quarter speed drive
			        if (leftDriveEncoder.Get() > 10*12*gDriveTicksPerInch)
			        	state = 10;
			        break;
				case 10:
					// make sure we winch back so we don't get a penalty
					if (i%2 == 0)
						printf("S10: done\n");
					if (kickTime.Get() > gWinchWaitTime && AutoArmKicker(kickPowerNear) && haveTime.Get() > 0.25)
					{
						FireKicker();
					}
		        	myRobot.ArcadeDrive(0.0, 0.0, false);
					break;
				}
			}

			Wait(0.04); // motor update time
		}

		autoController.Disable();
	}

	void OperatorControl(void)
	{
		DriverStationEnhancedIO &dseio = DriverStation::GetInstance()->GetEnhancedIO();
		GetWatchdog().SetEnabled(false);
		double aimAngle = 0;
		//int on = 1;
		//int off = 0;
		double leftDistance = 0;
		double rightDistance = 0;
		Timer shiftTime;
		int retraction = 0; // to be set
		int winchTarget = 0;
		bool prevFrontLimitSwitch = frontLimitSwitch.Get();
		int i =0;
		int kickSetting = 0;
		int lightNum = 0;
		bool raw10Previous = false;
		bool raw11Previous = false;
		double intakeVelocity = -.75;
		bool highGear = false;
		Timer haveTime;
		int* currentKickingPower = &midRetract;
		midRetract = 50;
		farRetract = 188 + 66;
		nearRetract = 0;
		int currentRetract = 0;

		shiftTime.Start();
		haveTime.Start();

		// we REALLY don't want this enabled here
		autoController.Disable();

		while (IsOperatorControl())
		{
			GetWatchdog().Feed();
			//sendIOPortData(&shifter);
			
			//if (dseio.IsUsable())
			{
				dseio.SetDigitalConfig(1, DriverStationEnhancedIO::kOutput);
				dseio.SetDigitalConfig(3, DriverStationEnhancedIO::kOutput);
				dseio.SetDigitalConfig(5, DriverStationEnhancedIO::kOutput);
				dseio.SetDigitalOutput(1, HaveBall());
				dseio.SetDigitalOutput(3, 0);
			//	dseio.SetDigitalOutput(1, highGear);
			}

			if (!HaveBall())
				haveTime.Reset();

			if (shiftTime.Get() > 0.3)
			{
				shifter1.Set(false);
				shifter2.Set(false);
			}
			if (kickPneumaticTime.Get() > 0.3)
			{
				ClearRatchetPneumatics();
			}

			i++;
			leftDistance = leftDriveEncoder.GetDistance();
			rightDistance = rightDriveEncoder.GetDistance();
						
		/*	if (stick3.GetTop())
			{
				printf("%f",leftDistance);
				printf("%f",rightDistance);
			} */
			
			// Helpful debugging info
			if (i%20 == 0)
			{
				printf("TELEOP\n");
				printf("L:%i R:%i\n", leftDriveEncoder.Get(), rightDriveEncoder.Get());
				printf("K: %i\n", kickerEncoder.Get());
				printf("BL:%i FL:%i\n", backLimitSwitch.Get(), frontLimitSwitch.Get());
				printf("IC: %f\n", intakeMotor.GetOutputCurrent());
				printf("RF: %f   RR: %f\n", rightFrontMotor.GetOutputCurrent(), rightRearMotor.GetOutputCurrent());
				printf("LF: %f   LR: %f\n", leftFrontMotor.GetOutputCurrent(), leftRearMotor.GetOutputCurrent());
				printf("\n");
				printf("Winch back: %i\n", retraction);
			//	printf("Mid autoSetting: %i\n", midRetract);
			}
			if (i > 10000)
			{
				i=1;
			}
			
			
			// Set kicker strength
			// Tested, world champ code
			if (stick3.GetRawButton(3))
				retraction = 50 + 33;
			else if (stick3.GetRawButton(5))
				retraction = 188 + 66;//gWinchFull;
			else if (stick3.GetRawButton(4))
				retraction = 0;
			
			// Variable kicker strength (one ratchet notch at a time for testing)
			/*
			if (stick3.GetRawButton(10) && !raw10Previous)
			{
				currentRetract -= 33;
				if (kickSetting == 1)
				{
					midRetract -= 33;
				}
				else if (kickSetting == 2)
				{
					farRetract -= 33;
				}
				else if (kickSetting == 0)
				{
					nearRetract -= 33;
				}
			}
			else if (stick3.GetRawButton(11) && !raw11Previous)
			{
				if (kickSetting == 1)
				{
					midRetract += 33;
				}
				else if (kickSetting == 2)
				{
					farRetract += 33;
				}
				else if (kickSetting == 0)
				{
					nearRetract += 33;
				}
			}
			if (stick3.GetRawButton(3))
			{
				printf("%i\n",midRetract);
				retraction = midRetract;
				currentKickingPower = &midRetract;
				kickSetting = 1;
			}
			else if (stick3.GetRawButton(5))
			{
				retraction = farRetract;//gWinchFull;
				currentKickingPower = &farRetract;
				kickSetting = 2;
			}
			else if (stick3.GetRawButton(4))
			{
				retraction = nearRetract;
				currentKickingPower = &nearRetract;
				kickSetting = 0;
			}
			
			
			
			
			if (retraction < 0)
			{
				retraction = 0;
				if (kickSetting == 1)
				{
					midRetract += 33;
				}
				else if (kickSetting == 2)
				{
					farRetract += 33;
				}
				else if (kickSetting == 0)
				{
					nearRetract += 33;
				}
			}
			else if (retraction > gWinchFull)
			{
				retraction = gWinchFull;
				if (kickSetting == 1)
				{
					midRetract -= 33;
				}
				else if (kickSetting == 2)
				{
					farRetract -= 33;
				}
				else if (kickSetting == 0)
				{
					nearRetract -= 33;
				}
			}
			raw10Previous = stick3.GetRawButton(10);
			raw11Previous = stick3.GetRawButton(11);
			*/
			
			// Original debug code, was active in world champs
			if (stick3.GetRawButton(10) && !raw10Previous)
			{
				retraction -= 33;
			}
			else if (stick3.GetRawButton(11) && !raw11Previous)
			{
				retraction += 33;
			}
			if (retraction < 0)
			{
				retraction = 0;
			}
			else if (retraction > gWinchFull)
			{
				retraction = gWinchFull;
			}
			raw10Previous = stick3.GetRawButton(10);
			raw11Previous = stick3.GetRawButton(11);
			
			
			// Kicker arm and fire control
			if (!armed)
			{
				// wait after kick before starting to winch back
				if (kickTime.Get() > gWinchWaitTime)
				{
					winchTarget = retraction;
					if (IsWinchOnTarget(winchTarget))
					{
						// auto arming mode
						bool autoArm = false;//!dseio.GetDigital(15);

						// arm; this starts the unwinch
						if ((stick3.GetTop() && !autoArm) || autoArm)
						{
							ArmKicker();
						}
					}
				}
			}
			else // armed
			{
				// unwinch
				winchTarget = -gWinchOut;
				if(kickerEncoder.Get() < (winchTarget + gWinchDeadband))
					dseio.SetDigitalOutput(3, 1);
				// fire!
				if (kickTime.Get() > gBetweenKickTime
					&& kickerEncoder.Get() < (winchTarget + gWinchDeadband)
					&& stick3.GetTrigger())
				{
					FireKicker();
				}
			}

			// manual control override on winch
			if (stick3.GetRawButton(9))
			{
				winchControl.Disable();
				kickerMotor.Set(stick3.GetY());
				armed = false;
				retraction = 0;
				
				// Kicker auto-zero using front limit switch rising edge (pressed -> not pressed)
				// XXX: how does this interact with PID?
				if (frontLimitSwitch.Get() == 1 && prevFrontLimitSwitch == 0)
					kickerEncoder.Reset(); //zeroing kicker
				prevFrontLimitSwitch = frontLimitSwitch.Get();
			}
			else
			{
				// automatic winch control
				if (i%20 == 0)
				{
					printf("E: %i\n", kickerEncoder.Get());
					printf("T: %i\n", winchTarget);
					printf("BL: %i  TL: %i\n", (winchTarget-gWinchDeadband), (winchTarget+gWinchDeadband));
					printf("\n");
				}
				RunWinch(winchTarget);
			}

			// Drive control
			//if (!zoomControl.isEnabled())
			//{
				myRobot.TankDrive(lstick, rstick); // drive with arcade style (use right stick)
			//}
			
			// Shifter control
			if (rstick.GetTrigger())
			{
				shifter1.Set(true);
				shifter2.Set(false);
				shiftTime.Reset();
				highGear = true;
			} else if (lstick.GetTrigger()) {
				shifter1.Set(false);
				shifter2.Set(true);
				shiftTime.Reset();
				highGear = false;
			}

			if (rstick.GetTop())
			{
				ratchet1.Set(true);
				ratchet2.Set(false);
				kickPneumaticTime.Reset();
			} else if (lstick.GetTop()) {
				ratchet1.Set(false);
				ratchet2.Set(true);
				kickPneumaticTime.Reset();
			}

			// INTAKE MOTOR!
			if (stick3.GetRawButton(6))
				intakeVelocity = 1.0;	// 0.75 old
			else if (stick3.GetRawButton(7))
				intakeVelocity = -0.75;
			else if (stick3.GetRawButton(8))
				intakeVelocity = 0.0;
			//else if (rstick.GetRawButton(11))
			//	intakeVelocity = 0.75;

			// The motors are stopped when kicking; start them up again after timeout
			if (kickTime.Get() > 0.1)
			{
				if (haveTime.Get() > 2.0 && intakeVelocity < 0.0 && lstick.GetY() <= 0.0 && rstick.GetY() <= 0.0)
					intakeMotor.Set(-0.35);
				else
					intakeMotor.Set(intakeVelocity);
			}
			
			/*if (lstick.GetTrigger())
			{

				gyro.Reset();
				aimControl.SetSetpoint(aimCamera.PIDGet());
				printf("Setpoint is: %f",aimControl.GetSetpoint());
				printf("Gyro is: %f",gyro.PIDGet());				
				aimControl.Enable();
				GetWatchdog().SetEnabled(false);
				Wait(5);
				
				
				GetWatchdog().SetEnabled(true);
				aimControl.Disable();
			}*/
		/*	if (rstick.GetTrigger())
			{
				//float dist = Target::FindCircularTargets(AxisCamera::getInstance().GetImage())[0].m_majorRadius;
				//printf("Size: %f\n", dist);
				//dist = dist * 10000;
				//printf("Probable Distance: %fin\n\n\n",(dist-4868.88)/-22.16);
				zoomControl.SetSetpoint(100);
				zoomControl.Enable();
				//printf("Error is: %f",zoomControl.GetError());
			} else if (lstick.GetTrigger()) {
				zoomControl.Disable();
			} */
			//if (rstick.GetTop())
			//{
				//CameraTarget& cameraTarget = CameraTarget::getInstance();
				//float a = cameraTarget.getDistance();
				//printf("Value: %f\n",a);
				//printf("Size: %f\n\n\n",(a*-22.16)+4868.88);
			//}
					
		//	if (stick3.GetTrigger())
			//{
			//CameraTarget& cameraTarget = CameraTarget::getInstance();
/*			if(stick3.GetTrigger())
			{
				on = 0;
				off = 1;
				dseio.SetLEDs(31);
			}
			else
			{
				on = 1;
				off = 0;
				dseio.SetLEDs(0);
			}
			for(int n=1;n<=5;n++)
				dseio.SetDigitalOutput(n,off);
*/
			//if (cameraTarget.isTargetFound())
			//{
				//\aimAngle = cameraTarget.getAngle();
				//int reducedAngle = int((aimAngle+19)/4.22); //Maps angle to the range 0 to 2*(numaber of LEDs)-1
				//int led = reducedAngle/2 + 1;
				
				//if (aimAngle > -19 && aimAngle < 19)
				//{
					/*
					dseio.SetDigitalOutput(led,on);
					dseio.SetLED(led,on);
					if((reducedAngle)%2==1)
					{
						dseio.SetDigitalOutput(led + 1,on);
						dseio.SetLED(led +1,on);
					}
					*/
				//}
				/*	if (aimAngle > -19 && aimAngle < -12)
					{
						dseio.SetDigitalOutput(1,1);
					} else if (aimAngle >= -12 && aimAngle < -5)
					{
						dseio.SetDigitalOutput(2,1);
					} else if (aimAngle >= -5 && aimAngle <= 5)
					{
						dseio.SetDigitalOutput(3,1);
					} else if (aimAngle > 5 && aimAngle <= 12)
					{
						dseio.SetDigitalOutput(4,1);
					} else if (aimAngle > 12 && aimAngle < 19)
					{
						dseio.SetDigitalOutput(5,1);
					} */
		//		}
			//}
			
			Wait(0.04);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(Robot2010);

