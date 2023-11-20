#include "DriveTrain.h"
#include "RobotMap.h"
#include "Commands/DriveWithJoystick.h"
#include "WPILib.h"

#define P 1
#define I 0.01
#define D 0.2

//#define SPEED

#ifdef SIMULATION
#define DRIVE_ENCODER_TICKS 360
#else
#define DRIVE_ENCODER_TICKS 900
#endif
#define WHEEL_DIAMETER 3.0

#define TICKS_PER_INCH (DRIVE_ENCODER_TICKS/M_PI/WHEEL_DIAMETER)

DriveTrain::DriveTrain() : Subsystem("DriveTrain"),
		frontLeft(FRONTLEFT),   // slave  1
		frontRight(FRONTRIGHT), // master 4
		backLeft(BACKLEFT),     // master 2
		backRight(BACKRIGHT),    // slave  3
		gyro(3)
{
	InitDrive();

	frontRight.ConfigLimitMode(CANTalon::kLimitMode_SrxDisableSwitchInputs);
	backLeft.ConfigLimitMode(CANTalon::kLimitMode_SrxDisableSwitchInputs);
	frontRight.SetFeedbackDevice(CANTalon::QuadEncoder);
	backLeft.SetFeedbackDevice(CANTalon::QuadEncoder);
	gyro.Reset();

#ifdef SPEED
	SetControlMode(CANTalon::kSpeed);
	frontRight.SetPID(P,I,D);
	backLeft.SetPID(P,I,D);
#else
	SetControlMode(CANTalon::kPercentVbus);
#endif

	//frontRight.SetDebug(2);
	frontRight.ConfigEncoderCodesPerRev(TICKS_PER_INCH);
	backLeft.ConfigEncoderCodesPerRev(TICKS_PER_INCH);

	gearPneumatic = new DoubleSolenoid(GEARSHIFTID,0,1);
	SetLowGear();

	SetExpiration(0.2);

	Publish(true);
}
void DriveTrain::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	SetDefaultCommand(new DriveWithJoystick());
}
void DriveTrain::TankDrive(float left, float right) {
	frontLeft.Set(BACKLEFT);
	backLeft.Set(left);
	frontRight.Set(-right);
	backRight.Set(FRONTRIGHT);

	Publish(false);

	m_safetyHelper->Feed();
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
void DriveTrain::CustomArcade(float xAxis, float yAxis, float zAxis, bool squaredInputs) {
	//cout << "xAxis:" << xAxis << " yAxis:" << yAxis << endl;
	double left=0;
	double right=0;

	if (squaredInputs) {
		if (xAxis >= 0.0) {
			xAxis = (xAxis * xAxis);
		} else {
			xAxis = -(xAxis * xAxis);
		}
		if (yAxis >= 0.0) {
			yAxis = (yAxis * yAxis);
		} else {
			yAxis = -(yAxis * yAxis);
		}
	}

	if (zAxis != 0) {
		xAxis = zAxis;
		yAxis = -zAxis;
	}

	if (xAxis > 0.0) {
		if (yAxis > 0.0) {
			left = xAxis - yAxis;
			right = std::max(xAxis, yAxis);
		} else {
			left = std::max(xAxis, -yAxis);
			right = xAxis + yAxis;
		}
	} else {
		if (yAxis > 0.0) {
			left = -std::max(-xAxis, yAxis);
			right = xAxis + yAxis;
		} else {
			left = xAxis - yAxis;
			right = -std::max(-xAxis, -yAxis);
		}
	}
	if(inlowgear){
		right*=0.5;
		left*=0.5;
	}
	// Make sure values are between -1 and 1
#ifndef SPEED
	left = coerce(-1, 1, left);
	right = coerce(-1, 1, right);
#endif
	frontLeft.Set(BACKLEFT);
	backLeft.Set(left);
	frontRight.Set(-right);
	backRight.Set(FRONTRIGHT);

	Publish(false);
	m_safetyHelper->Feed();
}

float DriveTrain::coerce(float min, float max, float x) {
	if (x < min)
		x = min;
	else if (x > max)
		x = max;
	return x;
}

void DriveTrain::SetLowGear() {
	if(!inlowgear){
		gearPneumatic->Set(DoubleSolenoid::kReverse);
		cout << "Setting Low Gear"<<endl;
		inlowgear=true;
	}
}

void DriveTrain::SetHighGear() {
	if(inlowgear){
		gearPneumatic->Set(DoubleSolenoid::kForward);
		cout << "Setting High Gear"<<endl;
		inlowgear=false;
	}
}
void DriveTrain::InitDrive() {
	m_safetyHelper = std::make_unique<MotorSafetyHelper>(this);
	m_safetyHelper->SetSafetyEnabled(true);
}

void DriveTrain::SetExpiration(double timeout) {
  m_safetyHelper->SetExpiration(timeout);
}

double DriveTrain::GetExpiration() const {
  return m_safetyHelper->GetExpiration();
}

bool DriveTrain::IsAlive() const { return m_safetyHelper->IsAlive(); }

bool DriveTrain::IsSafetyEnabled() const {
  return m_safetyHelper->IsSafetyEnabled();
}

void DriveTrain::SetSafetyEnabled(bool enabled) {
  m_safetyHelper->SetSafetyEnabled(enabled);
}

void DriveTrain::DisableDrive() {
	backRight.Disable();
	backLeft.Disable();
	frontRight.Disable();
	frontLeft.Disable();
}

void DriveTrain::EnableDrive() {
	Reset();
	backRight.Enable();
	backLeft.Enable();
	frontRight.Enable();
	frontLeft.Enable();
}

void DriveTrain::GetDescription(std::ostringstream& desc) const {
  desc << "DriveTrain";
}

void DriveTrain::StopMotor() {
  backLeft.StopMotor();
  frontRight.StopMotor();
  m_safetyHelper->Feed();
}

void DriveTrain::Reset() {
    double d=GetDistance();
    cout << "DriveTrain::Reset previous travel="<<travel_distance<<" current distance="<<d<<" new travel="<<travel_distance+d<<endl;
    travel_distance +=d;
	frontRight.Reset();
	backLeft.Reset();
	//gyro.Reset();
}
void DriveTrain::Enable() {
	frontRight.Enable();
	backLeft.Enable();
	Publish(true);
}

void DriveTrain::Disable() {
	frontRight.Disable();
	backLeft.Disable();
	frontRight.Reset();
	backLeft.Reset();
	angle=0;
	Publish(true);
}

double DriveTrain::GetHeading() {
//#ifdef SIMULATION // need sign change since chassis pose is inverted
//	return -gyro.GetAngle();
//#else
	return gyro.GetAngle();
//#endif
}

void DriveTrain::SetControlMode(CANTalon::ControlMode controlMode) {
	mode=controlMode;
	frontRight.SetControlMode(controlMode);
	backLeft.SetControlMode(controlMode);
	frontLeft.SetControlMode(CANTalon::kFollower);
	backRight.SetControlMode(CANTalon::kFollower);
}

void DriveTrain::Publish(bool init) {
	if(init){
		frc::SmartDashboard::PutNumber("Travel", 0);
		frc::SmartDashboard::PutBoolean("HighGear", false);
		frc::SmartDashboard::PutNumber("Heading", 0);

	}else{
		frc::SmartDashboard::PutNumber("Heading", GetHeading());
		frc::SmartDashboard::PutNumber("Travel", GetTravelDistance());
		frc::SmartDashboard::PutBoolean("HighGear", !inlowgear);
	}
}

double DriveTrain::GetDistance() {
	double d1=-frontRight.GetPosition();
	double d2=backLeft.GetPosition();
	double x=0.5*(d1+d2);
	return round(x * 100) / 100.0;
}
double DriveTrain::GetRightDistance() {
	return -frontRight.GetPosition();
}
double DriveTrain::GetLeftDistance() {
	return backLeft.GetPosition();
}

double DriveTrain::GetTravelDistance() {
    return travel_distance+GetDistance();
}
