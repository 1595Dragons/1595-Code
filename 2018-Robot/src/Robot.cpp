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
			} else {
				// Right code for switch goes here
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
	}
	void TeleopPeriodic() {
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
		lDrive1->Set(ControlMode::Velocity, lPow);
		rDrive1->Set(ControlMode::Velocity, rPow);
		SmartDashboard::PutNumber("Motor velocity Left", lDrive1->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Motor velocity Right", rDrive1->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Desired Motor velocity Left", lPow);
		SmartDashboard::PutNumber("Desired Motor velocity Right", rPow);
		SmartDashboard::PutNumber("Angle", ahrs->GetYaw());
	}

};

START_ROBOT_CLASS(Robot)
