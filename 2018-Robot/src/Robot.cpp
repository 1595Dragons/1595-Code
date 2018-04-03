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
	AnalogInput * bottomSwitch = new AnalogInput(0);
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
	double driveAccel=800, driveMaxVel=800, distanceToZero, velSetpoint;
	double autoDelay;
	float disSetpoint, angleSetpoint, lastAngle;
	SendableChooser<std::string> chooser;
	const std::string driveForward = "driveForward";
	float driveForwardDis = 200;
	const std::string switchMid = "switchMid";
	float pivot1 = 90;
	float neutralDis = 210;
	const std::string scaleLeft = "scaleLeft";
	const std::string switchLeft = "swichLeft";
	int nearScaleDis1 = 252, nearScalePivot1 = 35, nearScaleDis2 = 36, nearScalePivot2=-11, nearScaleDis3=80;
	float neutralPivot = 90, farScaleDis1=192, farScaleDis2 = 70, farSwitchDis1 = 167, nearSwitchDis=55, switchDis=5;
	bool backwards;
	float scalePivot1=25, scaleDis1=72;
	float autoDropHeight;
	const std::string scaleRight = "scaleRight";
	const std::string switchRight = "switchRight";
	double distToEnc = 105.5, angleToEnc=21; //103.87 for pracice robot
	bool switchR, scaleR;
	bool turning, driving, turnR;
	bool crossing;
	double kDP, kDI, kDD, kDIZ, kGP, kGI, kGD, kGIZ;
	float iE, tE, curVal, pastVal, diE, dtE, pastDist;
	int autoStage = 0;
	Timer autoTimer;
	Preferences *prefs;
	AHRS *ahrs;
	//LiveWindow* lw = LiveWindow::GetInstance();
	std::string autoSelected;
	ofstream outputFile; //output file

	bool practiceRobot = true; double liftScale = 16;

	TalonSRX * test = new TalonSRX(0);
	void RobotInit() {
		prefs = Preferences::GetInstance();
		chooser.AddDefault(driveForward, driveForward);
		chooser.AddObject(switchMid, switchMid);
		chooser.AddObject(scaleLeft, scaleLeft);
		chooser.AddObject(scaleRight, scaleRight);
		chooser.AddObject(switchLeft, switchLeft);
		chooser.AddObject(switchRight, switchRight);
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
		lDrive1->SetSensorPhase(!practiceRobot);
		rDrive1->SetSensorPhase(true);

		//operator
		rIntake->Set(ControlMode::Follower, 8); //follow lIntake
		rIntake->SetInverted(true);
		arm->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,kTimeoutMs);
	//	arm->SetInverted(true);
		arm->SetSensorPhase(true);
		if(!practiceRobot){
			lift->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,kTimeoutMs);
		}
		else { lift->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,kTimeoutMs); }
		lift->SetInverted(true);
		lift->SetSensorPhase(true);

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

	void Disabled(){
	}

	void AutonomousInit(){
		ahrs->ZeroYaw();
		autoStage = 0;
		kDP=prefs->GetDouble("kDP", kDP);
		kDI=prefs->GetDouble("kDI", kDI);
		kDD=prefs->GetDouble("kDD", kDD);
		kDIZ=prefs->GetDouble("kDIZ", kDIZ);
		kGP=prefs->GetDouble("kGP", kGP);
		kGI=prefs->GetDouble("kGI", kGI);
		kGD=prefs->GetDouble("kGD", kGD);
		kGIZ=prefs->GetDouble("kGIZ", kGIZ);


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
			}
			if (gameData[1] == 'L') {
				scaleR = false;
				//pivot1 = -90; //switch on left side
			} else {
				scaleR = true; //switch on right side
			}
		}
		autoTimer.Start();

		if(switchR && autoSelected == switchLeft){ crossing = true; }
		else if(scaleR && autoSelected==scaleLeft){ crossing = true; }
		else if(!switchR && autoSelected==switchRight){ crossing = true; }
		else if(!scaleR && autoSelected==scaleRight){ crossing = true; }
		else { crossing = false; }

		if(autoSelected == scaleRight){
			scalePivot1 *= -1;
			nearScalePivot1 *= -1;
			nearScalePivot2*= -1;
		}
		if(autoSelected == scaleRight || autoSelected == switchRight){ neutralPivot *= -1; }


		lift->SetSelectedSensorPosition(300,0,kTimeoutMs);
	//	arm->SetSelectedSensorPosition(000,0,kTimeoutMs);

		if(autoSelected == switchMid){
			lift->SetSelectedSensorPosition(700,0,kTimeoutMs);
		}
		Wait(.1);
	}
	void AutonomousPeriodic(){
		if(autoTimer.Get() > autoDelay){
			if(autoSelected == driveForward){
				if(autoStage == 0){
					angleSetpoint = 90;
					disSetpoint = 200;
					driving = false;
					turning = true;
					backwards = true;
				}
				if(autoStage == 1){
					driving = false;
					turning = false;
				}

			}
			else if(autoSelected == switchMid){
				SmartDashboard::PutNumber("SwichMid", 1);
				lift->Set(ControlMode::MotionMagic, freeLiftCubePos+100);
				if((lift->GetSelectedSensorPosition(0)) > freeLiftCubePos){
					arm->Set(ControlMode::MotionMagic, switchPos);
				}
				if(!switchR){
					if(autoStage == 0){
						driving = true; turning = false; backwards = false; disSetpoint = 15;
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
						disSetpoint = 105;
					}
					if(autoStage == 3){
						lastAngle = angleSetpoint;
						angleSetpoint = 0;
						turning = true; driving = false; backwards =false;
					}
					if(autoStage == 4){
						disSetpoint = 95-15;
						turning = false; driving = true; backwards = false;
					}
					if(autoStage == 5){
						driving = false; turning = false;
						cubeGrabber->Set(DoubleSolenoid::kForward);
						puncher->Set(DoubleSolenoid::kForward);
					}
				}
				else{
					if(autoStage == 0){
						driving = true; turning = false; backwards = false; disSetpoint = 100;
					}
					if(autoStage == 1){
						turning = false; driving = false;
						cubeGrabber->Set(DoubleSolenoid::kForward);
						puncher->Set(DoubleSolenoid::kForward);
					}
				}


			}
			else if((autoSelected == scaleLeft || autoSelected == scaleRight) && !crossing){
				//scale near
				if(autoStage == 0){
					driving = true; turning = false; backwards = true;
					disSetpoint = nearScaleDis1;
					lift->Set(ControlMode::MotionMagic, 1500);
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
						angleSetpoint = nearScalePivot1;
						if(lDrive1->GetSelectedSensorPosition(0)/distToEnc > 30){
							cubeGrabber->Set(DoubleSolenoid::kForward);
							puncher->Set(DoubleSolenoid::kForward);
						}
					}
					if(autoStage == 3){
						turning = true; driving = false; backwards = false;
						angleSetpoint = nearScalePivot2;
						puncher->Set(DoubleSolenoid::kReverse);
						if(arm->GetSelectedSensorPosition(0) > 10000){
							lift->Set(ControlMode::MotionMagic, freeLiftPos);
						}
						else{ lift->Set(ControlMode::MotionMagic, 300); }
						arm->Set(ControlMode::MotionMagic, 0);
					}
					if(autoStage == 4){
						lIntake->Set(ControlMode::PercentOutput, .5);
						turning = false; driving = true; backwards = false;
						disSetpoint = nearScaleDis3;
					}
					if(autoStage == 5){
						driving = false; turning = false;
						Wait(.3);
						if(lift->GetSelectedSensorPosition(0) > 80){
							lift->Set(ControlMode::MotionMagic, 60);
						}
						else{
							cubeGrabber->Set(DoubleSolenoid::kReverse);
							Wait(.1);
							lift->Set(ControlMode::MotionMagic, freeLiftCubePos);
							Wait(.5);
							arm->Set(ControlMode::MotionMagic, switchPos);
							Wait(.3);
							autoStage++;
						}
					}
					if(autoStage == 6 && ((switchR && scaleR) || (!switchR && !scaleR))){
						driving = true; turning = false; backwards = false;
						disSetpoint = 5;
					}
					if(autoStage == 7){
						driving = false; turning = false;
						cubeGrabber->Set(DoubleSolenoid::kReverse);
						puncher->Set(DoubleSolenoid::kForward);
					}
					else if(autoStage == 6){
						driving = false; turning = false;
					}
			}
			else{
				//generic
				if(autoStage == 0){
					lift->Set(ControlMode::Position, freeLiftCubePos+150);
					angleSetpoint = 0;
					disSetpoint = neutralDis;
					turning = false; driving = true;
					if(autoSelected == scaleLeft || autoSelected == scaleRight){
						backwards = true;
					}
					else{ backwards = false; }
				}
				if(autoStage == 1){
					turning = true; driving = false;
					angleSetpoint = neutralPivot;
				}

				//scale far
				if((autoSelected == scaleLeft || autoSelected == scaleRight)&&crossing){
					if(autoStage == 2){ //drive required distance
						lift->Set(ControlMode::Position, 1500);
						turning = false; driving = true; backwards = true;
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
						if(lDrive1->GetSelectedSensorPosition(0)/distToEnc > 65){
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
				if(autoSelected == switchRight || autoSelected == switchLeft){
					if(autoStage == 2){ //drive necessary distance
						arm->Set(ControlMode::MotionMagic, switchPos);
						turning = false; driving = true; backwards = false;
						angleSetpoint = neutralPivot;
						if(crossing){disSetpoint = farSwitchDis1; }
						else{ disSetpoint = nearSwitchDis; }

					}
					if(autoStage == 3){ //turn to face switch
						turning = true; driving = false; backwards = false;
						angleSetpoint = 180;
						lastAngle = neutralPivot;
					}
					if(autoStage == 4){ //drive to switch
						turning= false; driving = true; backwards = false;
						disSetpoint = switchDis;
						angleSetpoint = 180;
					}
					if(autoStage == 5){ //drop cube
						turning = false; driving = false;
						cubeGrabber->Set(DoubleSolenoid::kForward);
						cubeGrabber->Set(DoubleSolenoid::kReverse);
					}
				}

			}

			if(turning){
				curVal = ahrs->GetYaw();
				iE = integrate(curVal, iE, angleSetpoint, kGI, kGIZ, true);
				tE = PIDify(pastVal, curVal, angleSetpoint, kGP, iE, kGD, true);
				if(tE > .6) {	tE = .6;	}
				else if (tE <= -.6){ tE = -.6;	}
				lDrive1->Set(ControlMode::Velocity, -tE*800);
				rDrive1->Set(ControlMode::Velocity, tE*800);

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
				if(autoTimer.Get()>.5){
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

				distanceToZero = 10*(lDrive1->GetSelectedSensorVelocity(0)^2/(driveAccel*2));
				if(fabs(distanceToZero) > fabs(disSetpoint - lDrive1->GetSelectedSensorPosition(0)/distToEnc) && velSetpoint < driveMaxVel){
					if(!backwards){ velSetpoint += driveAccel * .02; }
					else { velSetpoint -= driveAccel * .02; }
				}
				else if (fabs(distanceToZero < fabs(disSetpoint - lDrive1->GetSelectedSensorPosition(0)/distToEnc))){
					if(!backwards){ velSetpoint -= driveAccel * .02; }
					else { velSetpoint += driveAccel * .02; }
				}

				double curError = angleSetpoint - ahrs->GetYaw();
				if(curError > 180){
					curError = curError - 360;
				}
				else if(curError < -180){
					curError = curError+360;
				}
				if(!backwards){
					lDrive1->Set(ControlMode::Velocity, velSetpoint*(1-curError * .1));
					rDrive1->Set(ControlMode::Velocity, velSetpoint*(1+curError * .1));
				}
				else{
					lDrive1->Set(ControlMode::Velocity, velSetpoint*(1+curError * .1));
					rDrive1->Set(ControlMode::Velocity, velSetpoint*(1-curError * .1));
				}
/*
				rDrive1->Set(ControlMode::MotionMagic, disSetpoint*distToEnc);
				lDrive1->Set(ControlMode::MotionMagic, disSetpoint*distToEnc);
*/
				if(abs(lDrive1->GetSelectedSensorPosition(0) - disSetpoint*distToEnc) < 5){
					autoStage ++;
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
		arm->SetSelectedSensorPosition(0,0,kTimeoutMs);
		if(!practiceRobot){	lift->SetSelectedSensorPosition(300,0,kTimeoutMs); }
		else { lift->SetSelectedSensorPosition(300 * liftScale,0,kTimeoutMs); }

		arm->Config_kF(0,.0361, kTimeoutMs);  //.0361, .1, 0, 0
		arm->Config_kP(0, .1, kTimeoutMs);
		arm->Config_kI(0, 0, kTimeoutMs);
		arm->Config_kD(0, 0, kTimeoutMs);
		arm->ConfigMotionCruiseVelocity(20000, kTimeoutMs); //20000
		arm->ConfigMotionAcceleration(40000, kTimeoutMs); //40000

		if(!practiceRobot){
			lift->Config_kF(0,1.13,kTimeoutMs);
			lift->Config_kP(0,4,kTimeoutMs);
			lift->Config_kI(0,kLI,kTimeoutMs);
			lift->Config_kD(0,kLD,kTimeoutMs);
			lift->ConfigMotionCruiseVelocity(150, kTimeoutMs);
			lift->ConfigMotionAcceleration(1000, kTimeoutMs);
		}
		else{
			lift->Config_kF(0,1.13/liftScale,kTimeoutMs);
			lift->Config_kP(0,4/liftScale,kTimeoutMs);
			lift->Config_kI(0,kLI,kTimeoutMs);
			lift->Config_kD(0,kLD,kTimeoutMs);
			lift->ConfigMotionCruiseVelocity(150*liftScale, kTimeoutMs);
			lift->ConfigMotionAcceleration(1000*liftScale, kTimeoutMs);
		}
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

		if(!practiceRobot){
			liftPos = lift->GetSelectedSensorPosition(0);
		}
		else{ liftPos = lift->GetSelectedSensorPosition(0)/liftScale; }

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
		if(setLiftPos < 0) { setLiftPos = 0; } //put limits on what the lift can be set to
		else if(setLiftPos > 2700){ setLiftPos = 2700; }
	//	if(setArmPos < 0) { setArmPos = 0; } //put limits on what the arm can be set to
		if(setArmPos > 320000){ setArmPos = 320000; }


		//handle what to do with grabber and intake
		if(adjust(op->GetRawAxis(3)) > 0){
			cubeGrabber->Set(DoubleSolenoid::kForward);
			if(objective == 0){

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
				setLiftPos = 30;
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

		arm->Set(ControlMode::MotionMagic,setArmPos);
//		arm->Set(ControlMode::PercentOutput, adjust(op->GetRawAxis(1)));
		if(!practiceRobot){
			lift->Set(ControlMode::MotionMagic,setLiftPos);
		}
		else { lift->Set(ControlMode::MotionMagic,setLiftPos*liftScale); }


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
		SmartDashboard::PutNumber("Switch", bottomSwitch->GetVoltage());
	}
};

START_ROBOT_CLASS(Robot)
