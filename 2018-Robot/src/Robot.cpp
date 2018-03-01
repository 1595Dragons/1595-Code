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

using namespace nt;
class Robot: public IterativeRobot {
private:
	//drive
	TalonSRX * lDrive1 = new TalonSRX(11);
	TalonSRX * rDrive1 = new TalonSRX(5);
	TalonSRX * lDrive2 = new TalonSRX(10);
	TalonSRX * rDrive2 = new TalonSRX(6);
	Joystick * dr = new Joystick(0);
	float turn, throttle,lPow, rPow;
	double kVF, kVP, kVI, kVD, kLFMod;
	bool climbing = false, PTOBool = false;

	//climbing
	DoubleSolenoid * PTO = new DoubleSolenoid(0,1);
	DoubleSolenoid * stabilizer = new DoubleSolenoid(2,3);

	Joystick *op = new Joystick(1);
	//intake
	TalonSRX * lIntake = new TalonSRX(0);
	TalonSRX * rIntake = new TalonSRX(0);
	TalonSRX * lPivot = new TalonSRX(0);
	TalonSRX * rPivot = new TalonSRX(0);
	AnalogInput *lPot = new AnalogInput(0);
	AnalogInput *rPot = new AnalogInput(1);
	double kPivot= .5, lIntakePos, rIntakePos;
	double lIntakeWide, lIntakeNarrow, lIntakeIn, rIntakeWide, rIntakeNarrow, rIntakeIn;

	//lift
	TalonSRX * lift = new TalonSRX(0);
	TalonSRX * arm = new TalonSRX(0);
	DoubleSolenoid * cubeGrabber = new DoubleSolenoid(4,5);
	double kLP, kLI, kLD, kAP, kAI, kAD;
	int switchPos, scaleLiftPos, scaleArmPos, freeArmPos, freeLiftPos;
	int liftPos, armPos, setLiftPos, setArmPos;
	int objective; //0 = intake, 1 = scale, 2 = switch


	//auto
	float pivot1 = 45, dis1 = 50;
	double encToDist = 5;
	double kDP, kDI, kDD, kDIZ, kGP, kGI, kGD, kGIZ;
	float iE, tE, curVal, pastVal, diE, dtE, pastDist;
	int autoStage = 0;
	Timer autoTimer;
	Preferences *prefs;
	AHRS *ahrs;

	void RobotInit() {
		prefs = Preferences::GetInstance();

		//drive
		rDrive2->Set(ControlMode::Follower, 5);
		lDrive2->Set(ControlMode::Follower, 11);
		rDrive1->SetInverted(true);
		rDrive2->SetInverted(true);
		lDrive1->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
		rDrive1->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
		lDrive1->SetSensorPhase(true);
		rDrive1->SetSensorPhase(true);

		//operator
		rIntake->Set(ControlMode::Follower, 0); //follow lIntake

        try {
            ahrs = new AHRS(I2C::Port::kMXP);
        } catch (std::exception& ex ) {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }

	}


	void AutonomousInit(){
		ahrs->ZeroYaw();
		autoStage = 0;
		prefs->GetDouble("kDP", kDP);
		prefs->GetDouble("kDI", kDI);
		prefs->GetDouble("kDD", kDD);
		prefs->GetDouble("kDIZ", kDIZ);
		prefs->GetDouble("kGP", kGP);
		prefs->GetDouble("kGI", kGI);
		prefs->GetDouble("kGD", kGD);
		prefs->GetDouble("kGIZ", kGIZ);
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if (gameData.length() > 0) {
			if (gameData[0] == 'L') {
				// Left code for the switch goes here
				// Output goes to left motor
			} else {
				// Right code for switch goes here
				// Output goes to right motor
			}
		}
		autoTimer.Start();
	}
	void AutonomousPeriodic(){
		if(autoStage == 0){
			curVal = ahrs->GetYaw();
			iE = integrate(curVal, iE, pivot1, kGI, kGIZ, true);
			tE = PIDify(pastVal, curVal, pivot1, kGP, iE, kGD, true);
			lDrive1->Set(ControlMode::PercentOutput, tE);
			if(abs(pivot1-curVal)>1){
				autoTimer.Reset();
			}
			if(autoTimer.Get()>.5){	autoStage = 1;	}
		}

		if(autoStage == 1){
			curVal = ahrs->GetYaw();
			float distance = (rDrive1->GetSelectedSensorPosition(0)+lDrive1->GetSelectedSensorPosition(0))/2*encToDist;
			diE = integrate(distance, diE, dis1, kDI, kDIZ, false);
			float dTE = PIDify(pastDist, distance, dis1, kDP, diE, kDD, false);
			pastDist = distance;
			float tE = PIDify(pastVal, curVal, pivot1, kDP, 0, kDD, true);
			pastVal = curVal;
			if(dTE > 1){ dTE = 1; }
			lDrive1->Set(ControlMode::PercentOutput, (dTE * (1-tE)));
			rDrive1->Set(ControlMode::PercentOutput, (dTE * (1+tE)));
			if(fabs(distance - dis1) < 2){
				autoStage = 2;
			}
		}

		if(autoStage == 2){
			curVal = ahrs->GetYaw();
			iE = integrate(curVal, iE, 0, kGI, kGIZ, true);
			tE = PIDify(pastVal, curVal, 0, kGP, iE, kGD, true);
			rDrive1->Set(ControlMode::PercentOutput, tE);
			if(abs(0-curVal)>1){
				autoTimer.Reset();
			}
			if(autoTimer.Get()>.5){	autoStage = 3;	}
		}

		if(autoStage == 3){
			lDrive1->Set(ControlMode::PercentOutput, 0);
			rDrive1->Set(ControlMode::PercentOutput, 0);
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

		lDrive1->Config_kF(0,kVF*kLFMod, kTimeoutMs);
		lDrive1->Config_kP(0,kVP,kTimeoutMs);
		lDrive1->Config_kI(0,kVI,kTimeoutMs);
		lDrive1->Config_kD(0,kVD,kTimeoutMs);
		rDrive1->Config_kF(0,kVF,kTimeoutMs);
		rDrive1->Config_kP(0,kVP,kTimeoutMs);
		rDrive1->Config_kI(0,kVI,kTimeoutMs);
		rDrive1->Config_kD(0,kVD,kTimeoutMs);

		climbing = false;
	}
	void TeleopPeriodic() {
		if(dr->GetRawButton(5) && dr->GetRawButton(6)){
			climbing = true;
			stabilizer->Set(DoubleSolenoid::kReverse);
		}

		if(climbing){
			if(dr->GetRawButton(5)&&!dr->GetRawButton(6)){ PTO->Set(DoubleSolenoid::kForward); }
			if(dr->GetRawButton(6)&&!dr->GetRawButton(5)){ PTO->Set(DoubleSolenoid::kForward); }
			if(dr->GetRawButton(1)){ stabilizer->Set(DoubleSolenoid::kForward); }
			if(dr->GetRawButton(4)){ stabilizer->Set(DoubleSolenoid::kReverse); }
		}

		if(!climbing || !PTOBool){
			turn = adjust(dr->GetRawAxis(4)); //get the turn, scale it down
			if(turn < 0){ turn = -turn*turn/3;	}
			else { turn = turn*turn/3; }
			throttle = -adjust(dr->GetRawAxis(1)); //get the throttle
			if(throttle < 0){ throttle = -throttle*throttle;	}
			else { throttle = throttle*throttle; }
			lPow = throttle + turn; //simple arcade drive
			rPow = throttle - turn;
			//replace 1000 with whatever the max speed of the drive train is in native units
			lPow *= 1930;
			rPow *= 1930;
			if(lPow != 0){	lDrive1->Set(ControlMode::Velocity, lPow); }
			else { lDrive1->Set(ControlMode::Disabled, 0); }
			if(rPow != 0){	rDrive1->Set(ControlMode::Velocity, rPow); }
			else { rDrive1->Set(ControlMode::Disabled, 0); }
		}
		else{
			lDrive1->Set(ControlMode::PercentOutput, throttle);
			rDrive1->Set(ControlMode::PercentOutput, throttle);
		}

		//intake-does include driving part
		if(objective == 2){ lIntakePos = lIntakeIn; rIntakePos = rIntakeIn; }
		else if(op->GetRawAxis(3)>.5){ lIntakePos = lIntakeWide; rIntakePos = rIntakeWide; }
		else{ lIntakePos = lIntakeNarrow; rIntakePos = rIntakeNarrow; }
		if(!op->GetRawButton(5)){ lIntake->Set(ControlMode::PercentOutput, adjust(op->GetRawAxis(2))); }

		//arm and scale
		if(op->GetPOV() == 180){ objective = 2; }
		else if(op->GetPOV() == 270){ objective = 0; }
		else if(op->GetPOV() == 0){ objective = 1; }
		liftPos = lift->GetSelectedSensorPosition(0);
		armPos = arm->GetSelectedSensorPosition(0);
		if(objective == 0){
			if(armPos < freeArmPos){
				setArmPos = 0; setLiftPos = 0;
			}
			else{
				if(liftPos > freeLiftPos){ setArmPos = 0; setLiftPos=freeLiftPos+500; }
				else{ setLiftPos = freeLiftPos + 500; setArmPos = freeArmPos; }
			}
		}
		else if(objective == 1){
			if(armPos > freeArmPos || liftPos > freeLiftPos){
				setArmPos = scaleArmPos;
				setLiftPos = scaleLiftPos;
			}
			else{
				setLiftPos = scaleLiftPos;
			}
		}
		else if(objective == 2){
			if(liftPos > freeLiftPos){
				setLiftPos = freeLiftPos+500;
				setArmPos = switchPos;
			}
			else{
				setLiftPos = freeLiftPos+500;
				setArmPos = freeArmPos+500;
			}
			if(armPos > freeLiftPos){
				setArmPos = switchPos;
				setLiftPos = 0;
			}
		}

		if(op->GetRawAxis(3)>.5){ cubeGrabber->Set(DoubleSolenoid::kForward); }
		if(op->GetRawButton(6)){ cubeGrabber->Set(DoubleSolenoid::kReverse); }

		if(abs(liftPos) > 1000){	lift->Set(ControlMode::Position, setLiftPos);	}
		else{ lift->Set(ControlMode::Disabled, 0); }
		if(abs(armPos) > 1000){ arm->Set(ControlMode::Position, setArmPos); }
		else{ arm->Set(ControlMode::Disabled, 0); }

		lIntake->Set(ControlMode::PercentOutput, kPivot*(lPot->GetVoltage()-lIntakePos));
		rIntake->Set(ControlMode::PercentOutput, kPivot*(rPot->GetVoltage()-rIntakePos));


		SmartDashboard::PutNumber("Motor velocity Left", lDrive1->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Motor velocity Right", rDrive1->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Desired Motor velocity Left", lPow);
		SmartDashboard::PutNumber("Desired Motor velocity Right", rPow);
		SmartDashboard::PutNumber("Angle", ahrs->GetYaw());
	}

};

START_ROBOT_CLASS(Robot)
