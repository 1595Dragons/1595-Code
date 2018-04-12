/**
 * Example demonstrating the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 *
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the reverseSensor() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.
 *
 * Tweak the PID gains accordingly.
 */
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "Constants.h"
#include "Functions.h"
#include "AHRS.h"
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
using namespace nt;
using namespace std;
class Robot: public IterativeRobot {
private:
	//drive
	TalonSRX * lDrive1 = new TalonSRX(10);
	TalonSRX * rDrive1 = new TalonSRX(3);
	TalonSRX * lDrive2 = new TalonSRX(9);
	TalonSRX * rDrive2 = new TalonSRX(2);
	Joystick * dr = new Joystick(0);
	float turn, throttle,lPow, rPow;
	double kVF=.6, kVP=.6, kVI=.005, kVD=10, kLFMod=.92;
	bool climbing = false, PTOBool = false;

	//climbing
	DoubleSolenoid * PTO = new DoubleSolenoid(0,1);
	DoubleSolenoid * climberBrake = new DoubleSolenoid(6,7);

	Joystick *op = new Joystick(1);

	//intake
	TalonSRX * lIntake = new TalonSRX(8);
	TalonSRX * rIntake = new TalonSRX(7);

	//lift
	TalonSRX * lift = new TalonSRX(1);
	TalonSRX * arm = new TalonSRX(11);
	DoubleSolenoid * cubeGrabber = new DoubleSolenoid(4,5);
	DoubleSolenoid * puncher = new DoubleSolenoid(3,2);
	DigitalInput * bottomSwitch = new DigitalInput(0);
	DigitalInput * armSwitch = new DigitalInput(1);
	Timer grabTimer;
	double kLP, kLI, kLD, kLIZ, kAP, kAI, kAD, kAIZ;
	int switchPos = 110000, scaleLiftPos=1250, scaleArmPos = 270000, freeLiftPos=450, freeLiftCubePos=1250;
	int liftPos, armPos, setLiftPos=0, setArmPos=0; //110000, 300000
	double armPow;
	int objective; //0 = intake, 1 = scale, 2 = switch
	bool grab; //whether or not we are grabbing cube
	bool trigger; //to tell when the thing is updated so lift can be manually controlled
	bool grabTrigger;
	//auto
	double driveAccel=1200, driveMaxVel=1200, distanceToZero, velSetpoint;
	double autoDelay;
	float disSetpoint, angleSetpoint, lastAngle;
	SendableChooser<std::string> chooser;
	const std::string driveForward = "driveForward";
	float driveForwardDis = 200;
	const std::string switchMid = "switchMid";
	float pivot1 = 90;
	float neutralDis = 220;
	float centerSwitchTurn = 60;
	const std::string scaleLeft = "scaleLeft";
	const std::string switchLeft = "swichLeft";
	int nearScaleDis1 = 240, nearScalePivot1 = 25, nearScaleDis2 = 52, nearScalePivot2=-8, nearScaleDis3=70;
	float neutralPivot = 90, farScaleDis1=210, farScaleDis2 = 64, farSwitchDis1 = 170, nearSwitchDis=46, switchDis=5;
	bool backwards;
	const std::string adaptiveRight = "adaptiveRight";
	const std::string adaptiveLeft = "adaptiveLeft";
	float scalePivot1=25, scaleDis1=72;
	float autoDropHeight;
	const std::string scaleRight = "scaleRight";
	const std::string switchRight = "switchRight";
	const std::string testTurn = "testTurn";
	double distToEnc = 100, angleToEnc=21; //103.87 for pracice robot
	bool switchR, scaleR;
	bool turning, driving, turnR;
	bool crossing;
	double kDP, kDI, kDD, kDIZ, kGP, kGI, kGD, kGIZ;
	float iE, tE, curVal, pastVal, diE, dtE, pastDist;
	int autoStage = 0;
	Timer autoTimer;
	Timer delayTimer;
	Preferences *prefs;
	AHRS *ahrs;
	//LiveWindow* lw = LiveWindow::GetInstance();
	std::string autoSelected;
	ofstream outputFile; //output file

	bool practiceRobot = false; double liftScale = 16;

	void RobotInit() {
		prefs = Preferences::GetInstance();
		chooser.AddDefault(driveForward, driveForward);
		chooser.AddObject(switchMid, switchMid);
		chooser.AddObject(scaleLeft, scaleLeft);
		chooser.AddObject(scaleRight, scaleRight);
		chooser.AddObject(switchLeft, switchLeft);
		chooser.AddObject(switchRight, switchRight);
		chooser.AddObject(adaptiveLeft, adaptiveLeft);
		chooser.AddObject(adaptiveRight, adaptiveRight);
		chooser.AddObject(testTurn, testTurn);
		SmartDashboard::PutData("Auto Modes", &chooser);

		//drive
		rDrive2->Set(ControlMode::Follower, 3);
		lDrive2->Set(ControlMode::Follower, 10);
		lDrive1->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		lDrive2->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		rDrive1->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		rDrive2->ConfigContinuousCurrentLimit(40, kTimeoutMs);
		rDrive1->SetInverted(true);
		rDrive2->SetInverted(true);
		lDrive1->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,kTimeoutMs);
		rDrive1->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,kTimeoutMs);
		lDrive1->SetSensorPhase(true);
		rDrive1->SetSensorPhase(true);

		//operator
		rIntake->Set(ControlMode::Follower, 8); //follow lIntake
		rIntake->SetInverted(true);
		arm->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,kTimeoutMs);
	//	arm->SetInverted(true);
		arm->SetSensorPhase(true);
	//	if(!practiceRobot){
			lift->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,kTimeoutMs);
	//	}
	//	else { lift->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,kTimeoutMs); }
		lift->SetInverted(false);
		lift->SetSensorPhase(false);

        try {
            ahrs = new AHRS(I2C::Port::kMXP);
        } catch (std::exception& ex ) {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }


//        outputFile.open("/media/sda1/curve.csv"); //create .CSV to output to
//        outputFile << "Time,lPosition,rPosition,lVel,rVel" << endl;
	}

	void DisabledPeriodic(){
		SmartDashboard::PutNumber("Angle", ahrs->GetYaw());
	}

	void AutonomousInit(){
		ahrs->ZeroYaw();
		autoStage = 0;


		autoDelay = prefs->GetDouble("autoDelay", autoDelay);



		arm->Config_kF(0,.0361, kTimeoutMs);  //.0361, .1, 0, 0
		arm->Config_kP(0, .1, kTimeoutMs);
		arm->Config_kI(0, 0, kTimeoutMs);
		arm->Config_kD(0, 0, kTimeoutMs);
		arm->ConfigMotionCruiseVelocity(20000, kTimeoutMs); //20000
		arm->ConfigMotionAcceleration(40000, kTimeoutMs); //40000

		lift->Config_kF(0,1.13,kTimeoutMs);
		lift->Config_kP(0,4,kTimeoutMs);
		lift->Config_kI(0,kLI,kTimeoutMs);
		lift->Config_kD(0,kLD,kTimeoutMs);
		lift->ConfigMotionCruiseVelocity(150, kTimeoutMs);
		lift->ConfigMotionAcceleration(1000, kTimeoutMs);

		lDrive1->Config_kF(0,kVF*kLFMod, kTimeoutMs);
		lDrive1->Config_kP(0,kVP,kTimeoutMs);
		lDrive1->Config_kI(0,kVI,kTimeoutMs);
		lDrive1->Config_kD(0,kVD,kTimeoutMs);
		rDrive1->Config_kF(0,kVF,kTimeoutMs);
		rDrive1->Config_kP(0,kVP,kTimeoutMs);
		rDrive1->Config_kI(0,kVI,kTimeoutMs);
		rDrive1->Config_kD(0,kVD,kTimeoutMs);
		/*
		lDrive1->Config_kF(0,.3, kTimeoutMs);
		lDrive1->Config_kP(0,3,kTimeoutMs);
		lDrive1->Config_kI(0,.1,kTimeoutMs);
		lDrive1->Config_IntegralZone(0,2,kTimeoutMs);
		lDrive1->Config_kD(0,0,kTimeoutMs);
		lDrive1->ConfigMotionCruiseVelocity(900, kTimeoutMs);
		lDrive1->ConfigMotionAcceleration(600, kTimeoutMs);

		rDrive1->Config_kF(0,.3, kTimeoutMs);
		rDrive1->Config_kP(0,3,kTimeoutMs);
		rDrive1->Config_kI(0,.15,kTimeoutMs);
		rDrive1->Config_IntegralZone(0,2,kTimeoutMs);
		rDrive1->Config_kD(0,0,kTimeoutMs);
		rDrive1->ConfigMotionCruiseVelocity(900, kTimeoutMs);
		rDrive1->ConfigMotionAcceleration(600, kTimeoutMs);
*/
		lDrive1->SetSelectedSensorPosition(0,0,kTimeoutMs);
		rDrive1->SetSelectedSensorPosition(0,0,kTimeoutMs);
		iE = 0;
		autoStage = 0;

		autoSelected = chooser.GetSelected(); //choose auto.
		//std::cout << "Auto selected: " << autoSelected << std::endl;
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData[0] == 'L') {
				switchR = false;
				//pivot1 = -90; //switch on left side
			} else {
				switchR = true; //switch on right side
				centerSwitchTurn *= -1;
			}
			if (gameData[1] == 'L') {
				scaleR = false;
				//pivot1 = -90; //switch on left side
			} else {
				scaleR = true; //switch on right side
			}
		}
		autoTimer.Start();

		if(autoSelected == adaptiveLeft){
			if(!scaleR){ autoSelected = scaleLeft; }
		//	else if(!switchR){ autoSelected = switchLeft; }
		//	else{ autoSelected = driveForward; }
			else { autoSelected = switchLeft; }
		}
		if(autoSelected == adaptiveRight){
			if(scaleR){ autoSelected = scaleRight; }
			//else if(switchR){ autoSelected = switchRight; }
			//else{ autoSelected = driveForward; }
			else { autoSelected = switchRight; }
		}


		if(switchR && autoSelected == switchLeft){ crossing = true; }
		else if(scaleR && autoSelected==scaleLeft){ crossing = true; }
		else if(!switchR && autoSelected==switchRight){ crossing = true; }
		else if(!scaleR && autoSelected==scaleRight){ crossing = true; }
		else { crossing = false; }

		if(autoSelected == testTurn || ((autoSelected == scaleRight|| autoSelected==scaleLeft)&&!crossing)){
			kGP=prefs->GetDouble("kGP", kGP);
			kGI=prefs->GetDouble("kGI", kGI);
			kGD=prefs->GetDouble("kGD", kGD);
			kGIZ=prefs->GetDouble("kGIZ", kGIZ);
		}
		else{
			kGP = .022; kGI = .003; kGD = .11; kGIZ = 5;
		}

		if(autoSelected == scaleRight){
			scalePivot1 *= -1;
			nearScalePivot1 *= -1;
			nearScalePivot2*= -1;
		}
		if(autoSelected == scaleLeft || autoSelected == switchLeft){ neutralPivot *= -1; }



	/*	if(practiceRobot){
			freeLiftCubePos *= liftScale; freeLiftPos *= liftScale;
			lift->Config_kF(0,1.13/liftScale,kTimeoutMs);
			lift->Config_kP(0,4/liftScale,kTimeoutMs);
			lift->Config_kI(0,kLI,kTimeoutMs);
			lift->Config_kD(0,kLD,kTimeoutMs);
			lift->ConfigMotionCruiseVelocity(150*liftScale, kTimeoutMs);
			lift->ConfigMotionAcceleration(1000*liftScale, kTimeoutMs);
		}*/
	//	if(autoSelected == switchMid){
	//		lift->SetSelectedSensorPosition(700,0,kTimeoutMs);
	//	}
		lift->SetSelectedSensorPosition(880,0,kTimeoutMs);
				arm->SetSelectedSensorPosition(000,0,kTimeoutMs);
		Wait(.1);
		delayTimer.Start();
	}
	void AutonomousPeriodic(){
		if(!bottomSwitch->Get()){
			lift->SetSelectedSensorPosition(0,0,kTimeoutMs);
		}
		if(delayTimer.Get() > autoDelay){
			if(autoSelected == testTurn){
				if(autoStage == 0){
					angleSetpoint = 30;
					driving = false;
					turning = true;
					backwards = false;
				}
				if(autoStage == 1){
					turning = false;
					driving = false;
				}
			}
			else if(autoSelected == driveForward){
				SmartDashboard::PutBoolean("Drive forwrd", true);
				if(autoStage == 0){
					angleSetpoint = 0;
					disSetpoint = 200;
					driving = true;
					turning = false;
					backwards = true;
				}
				if(autoStage == 1){
					driving = false;
					turning = false;
				}

			}
			else if(autoSelected == switchMid){
				SmartDashboard::PutNumber("SwichMid", 1);

				if(!switchR && autoStage < 6){
					lift->Set(ControlMode::MotionMagic, freeLiftCubePos+150);
					if(lift->GetSelectedSensorPosition(0) > freeLiftCubePos){
						arm->Set(ControlMode::MotionMagic, switchPos);
					}
					if(autoStage == 0){
						driving = true; turning = false; backwards = false; disSetpoint = 40;
						angleSetpoint = 0;
					}
					if(autoStage == 1){
						arm->Set(ControlMode::MotionMagic, switchPos);
						lastAngle = 0;
						angleSetpoint = -90;
						turning = true; driving = false; backwards = false;
					}
					if(autoStage == 2){
						//arm->Set(ControlMode::Position, switchPos);
						turning = false; driving = true; backwards = false;
						disSetpoint = 110;
					}
					if(autoStage == 3){
						lastAngle = angleSetpoint;
						angleSetpoint = 0;
						turning = true; driving = false; backwards =false;
					}
					if(autoStage == 4){
						disSetpoint = 100-40;
						turning = false; driving = true; backwards = false;
					}
					if(autoStage == 5){
						driving = false; turning = false;
						autoStage++;
						cubeGrabber->Set(DoubleSolenoid::kForward);
						puncher->Set(DoubleSolenoid::kForward);
					}
				}
				else{
					if(autoStage == 0){
						driving = true; turning = false; backwards = false; disSetpoint = 100; angleSetpoint = 0;
						lift->Set(ControlMode::MotionMagic, freeLiftCubePos + 150);
						if(lift->GetSelectedSensorPosition(0) > freeLiftCubePos){
							arm->Set(ControlMode::MotionMagic, switchPos);
						}
					}
					if(autoStage == 1){
						turning = false; driving = false;
						cubeGrabber->Set(DoubleSolenoid::kForward);
						puncher->Set(DoubleSolenoid::kForward);
						autoStage ++;
					}
					if(autoStage == 2){ autoStage = 6; }
				}
				if(autoStage == 6){
					arm->Set(ControlMode::MotionMagic, 0);
					lift->Set(ControlMode::MotionMagic, freeLiftPos);
					driving = true; turning = false; backwards = true; disSetpoint = 45;
				}
				if(autoStage == 7){
					lIntake->Set(ControlMode::PercentOutput, .5);
					puncher->Set(DoubleSolenoid::kReverse);
					turning = true; driving = false; angleSetpoint = centerSwitchTurn;
				}
				if(autoStage == 8){
					lift->Set(ControlMode::MotionMagic, 300);
					turning = false; driving = true; disSetpoint = 40; backwards = false;
				}
				if(autoStage == 9){
					Wait(1);
					lIntake->Set(ControlMode::PercentOutput, 0);
					lift->Set(ControlMode::MotionMagic, 20);
					Wait(.5);
					cubeGrabber->Set(DoubleSolenoid::kReverse);
					Wait(.2);
					lift->Set(ControlMode::MotionMagic, freeLiftCubePos+150);
					autoStage++;
				}
				if(autoStage == 10){
					turning = false; driving = true; disSetpoint = 40; backwards = true;
				}
				if(autoStage == 11){
					arm->Set(ControlMode::MotionMagic, switchPos);
					turning = true; driving = false; angleSetpoint = 0;
				}
				if(autoStage == 12){
					turning = false; driving = true; backwards = false; disSetpoint = 40;
				}
				if(autoStage == 13){
					turning = false; driving = false;
					cubeGrabber->Set(DoubleSolenoid::kForward);
					puncher->Set(DoubleSolenoid::kForward);
				}

			}
			//else if((autoSelected == scaleLeft || autoSelected == scaleRight || ((autoSelected == switchRight || autoSelected == switchLeft)&&switchR == scaleR)) && !crossing){
			else if((autoSelected == scaleLeft || autoSelected == scaleRight) && !crossing){
				//scale near
				if(autoStage == 0){
					driving = true; turning = false; backwards = true;
					disSetpoint = nearScaleDis1;
					lift->Set(ControlMode::MotionMagic, 2500);
					if(lift->GetSelectedSensorPosition(0) > freeLiftCubePos){
						arm->Set(ControlMode::MotionMagic, scaleArmPos);
					}
				}
					if(autoStage == 1){
						turning = true; driving = false; backwards = true;
						angleSetpoint = nearScalePivot1;
					}
					if(autoStage == 2){
						turning = false; driving = true; backwards = true;
						disSetpoint = nearScaleDis2;
					}
					if(autoStage == 3){
						cubeGrabber->Set(DoubleSolenoid::kForward);
						puncher->Set(DoubleSolenoid::kForward);
						Wait(.2);
						autoStage ++;
					}
					if(autoStage == 4){
						turning = true; driving = false; backwards = false;
						angleSetpoint = nearScalePivot2;

						arm->Set(ControlMode::MotionMagic, 0);
						if(arm->GetSelectedSensorPosition(0) > 10000){
							lift->Set(ControlMode::MotionMagic, freeLiftPos);
							puncher->Set(DoubleSolenoid::kReverse);
						}
						else{ lift->Set(ControlMode::MotionMagic, 300); }

					}
					if(autoStage == 5){
						lIntake->Set(ControlMode::PercentOutput, .8);
						turning = false; driving = true; backwards = false;
						disSetpoint = nearScaleDis3;
						arm->Set(ControlMode::MotionMagic, 0);
						if(arm->GetSelectedSensorPosition(0) > 10000){
							lift->Set(ControlMode::MotionMagic, freeLiftPos+150);
						}
						else{ lift->Set(ControlMode::MotionMagic, 300); }
					}
					if(autoStage == 6){
						driving = false; turning = false;
						lDrive1->Set(ControlMode::PercentOutput, .4);
						Wait(.2);
						lDrive1->Set(ControlMode::PercentOutput, -.4);
						Wait(.2);
						lDrive1->Set(ControlMode::PercentOutput, .4);
						Wait(.2);
						lDrive1->Set(ControlMode::PercentOutput, -.4);
						Wait(.2);
						lDrive1->Set(ControlMode::PercentOutput, 0);
						lift->Set(ControlMode::MotionMagic,20);
						Wait(.4);
						cubeGrabber->Set(DoubleSolenoid::kReverse);
						Wait(.2);
						lift->Set(ControlMode::MotionMagic, 2000);
						autoStage++;
					}

					if((autoSelected == scaleRight|| autoSelected == scaleLeft) && autoStage > 5){
						if(lift->GetSelectedSensorPosition(0) > freeLiftCubePos){
							arm->Set(ControlMode::MotionMagic, scaleArmPos);
						}
						if(autoStage == 7){
							driving = true; turning = false; backwards = true;
							disSetpoint = nearScaleDis3-5;
						}
						if(autoStage == 8){
							if(arm->GetSelectedSensorPosition(0) > 265000){
								cubeGrabber->Set(DoubleSolenoid::kForward);
								puncher->Set(DoubleSolenoid::kForward);
								Wait(.2);
								autoStage ++;
							}


						}
						if(autoStage == 9){
							lift->Set(ControlMode::MotionMagic, freeLiftPos);
							arm->Set(ControlMode::MotionMagic, 0);
						}
					}

					if(autoSelected==switchRight || autoSelected==switchLeft){
						if(lift->GetSelectedSensorPosition(0) > freeLiftCubePos){
							arm->Set(ControlMode::MotionMagic, switchPos);
						}
						if(autoStage == 7){
							driving = false; turning = false;
							if(arm->GetSelectedSensorPosition(0) > 100000){
								cubeGrabber->Set(DoubleSolenoid::kForward);
								puncher->Set(DoubleSolenoid::kForward);
							}
						}
					}
			}
			else{
				//generic
				if(autoStage == 0){
					lift->Set(ControlMode::Position, freeLiftCubePos+150);
					angleSetpoint = 0;
					disSetpoint = neutralDis;
					turning = false; driving = true;
					backwards = true;
				}
				if(autoStage == 1){
					turning = true; driving = false;
					angleSetpoint = neutralPivot;
				}

				//scale far
				if((autoSelected == scaleLeft || autoSelected == scaleRight)&&crossing){
					if(autoStage == 2){ //drive required distance
						lift->Set(ControlMode::Position, 1500);
						turning = false; driving = true; backwards = false;
						disSetpoint = farScaleDis1;
						angleSetpoint = neutralPivot;
					}
					if(autoStage == 3){ //turn ot face scale
						turning = true; driving = false; backwards = true;
						lastAngle = angleSetpoint;
						angleSetpoint = 0;
					}
					if(autoStage == 4){ //drive forward
						turning = false; driving = true; backwards = true;
						angleSetpoint = 0;
						disSetpoint = farScaleDis2;
						if(lDrive1->GetSelectedSensorPosition(0)/distToEnc > 60){
							cubeGrabber->Set(DoubleSolenoid::kForward);
							puncher->Set(DoubleSolenoid::kForward);
						}
					}
					if(autoStage == 5){ //drop cube
						turning = false; driving = false;
						arm->Set(ControlMode::MotionMagic, 0);
						lift->Set(ControlMode::MotionMagic, freeLiftPos);
					}
				}

				//switch
			//	if(autoSelected == switchRight || autoSelected == switchLeft){
					if(autoStage == 2){ //drive necessary distance
						arm->Set(ControlMode::MotionMagic, switchPos);
						turning = false; driving = true; backwards = false;
						angleSetpoint = neutralPivot;
						if(crossing){disSetpoint = farSwitchDis1; } //THIS IS WRONG FIX THIS FIX THIS
						else{ disSetpoint = nearSwitchDis; }

					}
					if(autoStage == 3){ //turn to face switch
						turning = true; driving = false; backwards = false;
						angleSetpoint = 0;
						lastAngle = neutralPivot;
					}
					if(autoStage == 4){ //drive to switch
						turning= false; driving = true; backwards = false;
						disSetpoint = switchDis;
						angleSetpoint = 0;
					}
					if(autoStage == 5){ //drop cube
						turning = false; driving = false;
						cubeGrabber->Set(DoubleSolenoid::kForward);
						puncher->Set(DoubleSolenoid::kForward);
						lIntake->Set(ControlMode::PercentOutput,.5);
						lift->Set(ControlMode::MotionMagic, freeLiftPos+150);
						arm->Set(ControlMode::MotionMagic, 0);
						if(arm->GetSelectedSensorPosition(0) < 10000){
							puncher->Set(DoubleSolenoid::kReverse);
							Wait(.5);
							lift->Set(ControlMode::MotionMagic, 20);
							Wait(.5);
							cubeGrabber->Set(DoubleSolenoid::kReverse);
							Wait(.2);
							lift->Set(ControlMode::MotionMagic, freeLiftCubePos+150);
							autoStage++;
						}
					}
					if(autoStage == 6){
						if(lift->GetSelectedSensorPosition(0) > freeLiftCubePos){
							arm->Set(ControlMode::MotionMagic, switchPos);
							if(arm->GetSelectedSensorPosition(0)>100000){
								cubeGrabber->Set(DoubleSolenoid::kForward);
								puncher->Set(DoubleSolenoid::kForward);
							}
						}
				//	}
				}

			}

			if(turning){
				curVal = ahrs->GetYaw();
				iE = integrate(curVal, iE, angleSetpoint, kGI, kGIZ, true);
				tE = PIDify(pastVal, curVal, angleSetpoint, kGP, iE, kGD, true);
				if(tE > .6) {	tE = .6;	}
				else if (tE <= -.6){ tE = -.6;	}
				lDrive1->Set(ControlMode::Velocity, -tE*1500);
				rDrive1->Set(ControlMode::Velocity, tE*1500);

				double curError = angleSetpoint - curVal;
				if(curError > 180){
					curError = curError - 360;
				}
				else if(curError < -180){
					curError = curError+360;
				}

				if(fabs(curError)>1.5){
					autoTimer.Reset();
				}
				if(autoTimer.Get()>.125){
					autoStage++;
					lDrive1->SetSelectedSensorPosition(0,0,kTimeoutMs);
					rDrive1->SetSelectedSensorPosition(0,0,kTimeoutMs);
					Wait(.01);
				}
				pastVal = curVal;
				//
			}

			else if(driving){
				if(backwards){ disSetpoint *= -1; }
				float vel = lDrive1->GetSelectedSensorVelocity(0);
				distanceToZero = fabs(vel)/driveAccel*(10*vel/2)/distToEnc*1.5;

				double curError = angleSetpoint - ahrs->GetYaw();
				if(curError > 180){
					curError = curError - 360;
				}
				else if(curError < -180){
					curError = curError+360;
				}

				if(disSetpoint > lDrive1->GetSelectedSensorPosition(0)/distToEnc){
					if(velSetpoint < 0){
						velSetpoint = 0;
					}
					if(distanceToZero > (disSetpoint - lDrive1->GetSelectedSensorPosition(0)/distToEnc)){
						velSetpoint -= driveAccel *.03;
						lDrive1->Set(ControlMode::Velocity, (velSetpoint-70)*(1+curError * .1));
						rDrive1->Set(ControlMode::Velocity, (velSetpoint-70)*(1-curError * .1));
					}
					else if(velSetpoint < driveMaxVel){
						velSetpoint += driveAccel*.02;
						lDrive1->Set(ControlMode::Velocity, velSetpoint*(1+curError * .1));
						rDrive1->Set(ControlMode::Velocity, velSetpoint*(1-curError * .1));
					}
					else{
						lDrive1->Set(ControlMode::Velocity, velSetpoint*(1+curError * .1));
						rDrive1->Set(ControlMode::Velocity, velSetpoint*(1-curError * .1));
					}
					SmartDashboard::PutBoolean("Forward", true);

				}
				else{
					if(velSetpoint > 0){ velSetpoint = 0; }
					if(distanceToZero < (disSetpoint - lDrive1->GetSelectedSensorPosition(0)/distToEnc)){
						velSetpoint += driveAccel *.03;
						lDrive1->Set(ControlMode::Velocity, (velSetpoint+70)*(1-curError * .1));
						rDrive1->Set(ControlMode::Velocity, (velSetpoint+70)*(1+curError * .1));
					}
					else if(velSetpoint > -driveMaxVel){
						velSetpoint -= driveAccel*.02;
						lDrive1->Set(ControlMode::Velocity, velSetpoint*(1-curError * .1));
						rDrive1->Set(ControlMode::Velocity, velSetpoint*(1+curError * .1));
					}
					else{
						lDrive1->Set(ControlMode::Velocity, velSetpoint*(1-curError * .1));
						rDrive1->Set(ControlMode::Velocity, velSetpoint*(1+curError * .1));
					}

					SmartDashboard::PutBoolean("Forward", false);
				}
/*
				rDrive1->Set(ControlMode::MotionMagic, disSetpoint*distToEnc);
				lDrive1->Set(ControlMode::MotionMagic, disSetpoint*distToEnc);
*/
				if(velSetpoint > 20 || abs(lDrive1->GetSelectedSensorPosition(0) - disSetpoint*distToEnc) > 150){
					autoTimer.Reset();
				}
				if(abs(lDrive1->GetSelectedSensorPosition(0) - disSetpoint*distToEnc) < 10 || autoTimer.Get() > .5){
					autoStage ++;
					velSetpoint = 0;
					lDrive1->SetSelectedSensorPosition(0,0,kTimeoutMs);
					rDrive1->SetSelectedSensorPosition(0,0,kTimeoutMs);
		//			lDrive1->ConfigMotionAcceleration(800, kTimeoutMs);
		//			rDrive1->ConfigMotionAcceleration(800, kTimeoutMs);
		//			lDrive1->Config_kI(0,.03,kTimeoutMs);
		//			rDrive1->Config_kI(0,.03,kTimeoutMs);
					Wait(.01);
				}
			}

			else{
				lDrive1->Set(ControlMode::PercentOutput, 0);
				rDrive1->Set(ControlMode::PercentOutput, 0);
			}

			SmartDashboard::PutNumber("Autostage", autoStage);
			SmartDashboard::PutNumber("Ldist", lDrive1->GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("Angle", ahrs->GetYaw());
			SmartDashboard::PutNumber("Arm", arm->GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("Lift", lift->GetSelectedSensorPosition(0));
			SmartDashboard::PutNumber("tE", tE);
		}
	}
	/**
	 * This function is called periodically during operator control
	 */
	void TeleopInit(){
		kVF=prefs->GetDouble("kVF", kVF);
		kLFMod=prefs->GetDouble("kLFMod", kLFMod);
		kVP=prefs->GetDouble("kVP", kVP);
		kVI=prefs->GetDouble("kVI", kVI);
		kVD=prefs->GetDouble("kVD", kVD);

		climbing = false;
		//arm->SetSelectedSensorPosition(0,0,kTimeoutMs);
		//if(!practiceRobot){	lift->SetSelectedSensorPosition(300,0,kTimeoutMs); }
		//else { lift->SetSelectedSensorPosition(300 * liftScale,0,kTimeoutMs); }

		arm->Config_kF(0,.0361, kTimeoutMs);  //.0361, .1, 0, 0
		arm->Config_kP(0, .1, kTimeoutMs);
		arm->Config_kI(0, 0, kTimeoutMs);
		arm->Config_kD(0, 0, kTimeoutMs);
		arm->ConfigMotionCruiseVelocity(20000, kTimeoutMs); //20000
		arm->ConfigMotionAcceleration(40000, kTimeoutMs); //40000

//		if(!practiceRobot){
			lift->Config_kF(0,1.13,kTimeoutMs);
			lift->Config_kP(0,4,kTimeoutMs);
			lift->Config_kI(0,kLI,kTimeoutMs);
			lift->Config_kD(0,kLD,kTimeoutMs);
			lift->ConfigMotionCruiseVelocity(150, kTimeoutMs);
			lift->ConfigMotionAcceleration(1000, kTimeoutMs);
		//	lift->SetSelectedSensorPosition(740,0,kTimeoutMs);
/*		}
		else{
			lift->Config_kF(0,1.13/liftScale,kTimeoutMs);
			lift->Config_kP(0,4/liftScale,kTimeoutMs);
			lift->Config_kI(0,kLI,kTimeoutMs);
			lift->Config_kD(0,kLD,kTimeoutMs);
			lift->ConfigMotionCruiseVelocity(150*liftScale, kTimeoutMs);
			lift->ConfigMotionAcceleration(1000*liftScale, kTimeoutMs);
		}*/
	//	arm->Config_IntegralZone(0,kAIZ, kTimeoutMs);

		objective = 0; trigger = true;

		grabTimer.Start();
	}
	void TeleopPeriodic() {
		if(dr->GetRawButton(5) && dr->GetRawButton(6)){
			climbing = true;
			climberBrake->Set(DoubleSolenoid::kForward);
		}

		if(climbing){
			if(dr->GetRawButton(5)&&!dr->GetRawButton(6)){
				PTO->Set(DoubleSolenoid::kForward);
				PTOBool = false;
			}
			if(dr->GetRawButton(6)&&!dr->GetRawButton(5)){
				PTO->Set(DoubleSolenoid::kReverse);
				PTOBool = true;
			}
			if(dr->GetRawButton(1)){
				climberBrake->Set(DoubleSolenoid::kReverse);
			}
		}
		else { climberBrake->Set(DoubleSolenoid::kReverse); }

			turn = adjust(dr->GetRawAxis(4)); //get the turn, scale it down
		//	if(turn < 0){ turn = -turn*turn/3;	}
		//	else { turn = turn*turn/3; }
			turn/= 2;
			throttle = -adjust(dr->GetRawAxis(1)); //get the throttle
			if(throttle < 0){ throttle = -throttle*throttle;	}
			else { throttle = throttle*throttle; }
			lPow = throttle + turn; //simple arcade drive
			rPow = throttle - turn;
			lDrive1->Set(ControlMode::PercentOutput, lPow);
			rDrive1->Set(ControlMode::PercentOutput, rPow);
			//replace 1000 with whatever the max speed of the drive train is in native units
			/*lPow *= 1930;
			rPow *= 1930;
			lDrive1->Set(ControlMode::Velocity, lPow);
			rDrive1->Set(ControlMode::Velocity, rPow);
		*/

		//arm and scale
		if(op->GetPOV() == 180){ objective = 2; trigger = true; }
		else if(op->GetPOV() == 270){ objective = 0; trigger = true; }
		else if(op->GetPOV() == 0){ objective = 1; trigger = true; }
		else if(op->GetPOV() == 90){ objective = 3; trigger = true; }

	//	if(!practiceRobot){
			liftPos = lift->GetSelectedSensorPosition(0);
	//	}
	//	else{ liftPos = lift->GetSelectedSensorPosition(0)/liftScale; }

		armPos = arm->GetSelectedSensorPosition(0);
		if(trigger){
			if(objective == 0){ //intake
					puncher->Set(DoubleSolenoid::kReverse);
					if(armPos < 1000){ //if the arm is in position
						setArmPos = 0; //zero everything out
						setLiftPos = 300;
						trigger = false; //objective complete
					}
					else{
						if(liftPos > freeLiftPos){ //if we are able to move arm
							setArmPos = 0;			//move arm
							setLiftPos=freeLiftPos + 150; //and put lift as low as it can go
						}
						else {
							setLiftPos = freeLiftPos + 150; //get lift to spot so arm can move
						}
					}
			}
			else if(objective == 1){ //scale
				if(armPos > 20000 || liftPos > freeLiftCubePos){ //if everything is free to move
					setArmPos = scaleArmPos; //go to proper position
					setLiftPos = scaleLiftPos;
					trigger = false; //objective has command to be complete here
				}
				else{
					setLiftPos = scaleLiftPos+150; //otherwise raise lift till all is good
				}
			}
			else if(objective == 3){
				if(armPos > 20000){ setArmPos = scaleArmPos; setLiftPos = 300; trigger = false;}
				else if(liftPos > freeLiftCubePos){ setArmPos = scaleArmPos; }
				else { setLiftPos = freeLiftCubePos + 150; }
			}
			else if(objective == 2){ //switch
				if(liftPos > freeLiftCubePos){	setArmPos = switchPos;	} //if arm is free to go, move it
				else{ setLiftPos = freeLiftCubePos+150; } //otherwise make it free
				if(armPos > 20000){	 //if arm is above the necessary spot
					setArmPos = switchPos; //move to switch position
					setLiftPos = 0;
					trigger = false; //objective complete
				}
			}
		}

		if(trigger == false){
			setLiftPos -= adjust(op->GetRawAxis(5)) * 30; //if the thing is done moving autonomously, let operator control
			setArmPos -= adjust(op->GetRawAxis(1)) * 3000;
		}
		else if(setLiftPos > 2700){ setLiftPos = 2700; }
	//	if(setArmPos < 0) { setArmPos = 0; } //put limits on what the arm can be set to
		if(setArmPos > 320000){ setArmPos = 320000; }


		//handle what to do with grabber and intake
		if(adjust(op->GetRawAxis(3)) > 0){
			cubeGrabber->Set(DoubleSolenoid::kForward);
			if(objective == 0){
				setLiftPos = 300;
				lIntake->Set(ControlMode::PercentOutput, adjust(op->GetRawAxis(3))); grab = true;
			}
			else if(objective == 2){
				puncher->Set(DoubleSolenoid::kForward);
				cubeGrabber->Set(DoubleSolenoid::kForward);
			}
			else if(objective == 1){
				cubeGrabber->Set(DoubleSolenoid::kForward);
			}
			SmartDashboard::PutBoolean("Grab", false);
		}
		else if(adjust(op->GetRawAxis(2)) > 0){
			cubeGrabber->Set(DoubleSolenoid::kReverse);
			SmartDashboard::PutBoolean("Grab", true);
			lIntake->Set(ControlMode::PercentOutput, adjust(op->GetRawAxis(2)));
			setLiftPos = 0;
		}
		else if(op->GetRawButton(5) && objective == 1){
			puncher->Set(DoubleSolenoid::kForward); cubeGrabber->Set(DoubleSolenoid::kForward);
		}
		else if(op->GetRawButton(6)){
			lIntake->Set(ControlMode::PercentOutput, -1);
		}
		else{
			puncher->Set(DoubleSolenoid::kReverse);
			if(grab){
				setLiftPos = 0;
				if(liftPos < 40){
				//	cubeGrabber->Set(DoubleSolenoid::kForward);
					grab = false;
					cubeGrabber->Set(DoubleSolenoid::kReverse);
					SmartDashboard::PutBoolean("Grab", true);
					Wait(.2);
					setLiftPos = 300;
					grab = false;
				}
/*				if(!grabTrigger){
					grabTrigger = true;
					grabTimer.Reset();
				}
				if(grabTimer.Get() > 3){
					cubeGrabber->Set(DoubleSolenoid::kReverse);
					Wait(.2);
					setLiftPos = 300;
					grabTrigger = false;
				}*/
			}
			else{
				cubeGrabber->Set(DoubleSolenoid::kReverse);
				SmartDashboard::PutBoolean("Grab", true);
			}
			lIntake->Set(ControlMode::PercentOutput, 0);

		}

	//	lift->Set(ControlMode::PercentOutput, -adjust(op->GetRawAxis(5)));
	//	arm->Set(ControlMode::PercentOutput, -adjust(op->GetRawAxis(1)));

//		if(objective==1){ setArmPos = scaleArmPos; }
//		else if(objective == 2){ setArmPos = switchPos; }
//		else { setArmPos = 0; }

		if(!bottomSwitch->Get() && liftPos < 1000){
			lift->SetSelectedSensorPosition(0,0,kTimeoutMs);
			if(setLiftPos < 0){ setLiftPos = 0; }
		}
		if(op->GetRawButton(1)){
			arm->SetSelectedSensorPosition(0,0,kTimeoutMs);
			setArmPos = 0;
		}

		arm->Set(ControlMode::MotionMagic,setArmPos);
//		arm->Set(ControlMode::PercentOutput, adjust(op->GetRawAxis(1)));
	//	if(!practiceRobot){
			lift->Set(ControlMode::MotionMagic,setLiftPos);
	//	}
	//	else { lift->Set(ControlMode::MotionMagic,setLiftPos*liftScale); }


//		SmartDashboard::PutNumber("LPot", lPot->GetVoltage());
//		SmartDashboard::PutNumber("RPot", rPot->GetVoltage());
		SmartDashboard::PutNumber("Motor velocity Left", lDrive1->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Motor velocity Right", rDrive1->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Desired Motor velocity Left", lPow);
		SmartDashboard::PutNumber("Desired Motor velocity Right", rPow);
		SmartDashboard::PutNumber("Angle", ahrs->GetYaw());
		SmartDashboard::PutNumber("Arm", arm->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("arm current", arm->GetOutputCurrent());
		SmartDashboard::PutNumber("Lift", liftPos);
		SmartDashboard::PutNumber("Objective", objective);
		bool test = armSwitch->Get();
		SmartDashboard::PutBoolean("Switch", test);
	}
};

START_ROBOT_CLASS(Robot)
