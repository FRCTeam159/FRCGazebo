
#include <math.h>

#include "DriveTrain.h"
#include "Assignments.h"

#include "Commands/TankDriveWithJoystick.h"

#define WHEEL_DIAMETER 7.5
#define DEADBAND 0.25

#ifdef SIMULATION
#define WHEEL_TICKS 360 // default encoder ticks given in simulation
#else // replace these values for whatever is appropriate for real drive-train
#define WHEEL_TICKS 900
#endif
#define INVERT_RIGHT_SIDE true

#define MP 0.5
#define MI 0.00
#define MD 0.5

#define MAX_POS_ERROR 0.1 // tolerance for set position
DriveTrain::DriveTrain() : Subsystem("DriveTrain"),
	left_motor(DRIVE_LEFT),right_motor(DRIVE_RIGHT),gyro(DRIVE_ANGLE)
{
	std::cout<<"New DriveTrain("<<DRIVE_LEFT<<","<<DRIVE_RIGHT<<")"<<std::endl;
	left_motor.SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	right_motor.SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
	SetDistancePerPulse(WHEEL_DIAMETER,WHEEL_TICKS,INVERT_RIGHT_SIDE);
	SetDeadband(DEADBAND,DEADBAND);
	gyro.Reset();
	Log();
}

// ===========================================================================================================
// DriveTrain::SetDistancePerPulse(double d, double t, bool b)
// ===========================================================================================================
// inputs
//   d    wheel diameter (inches)
//   t    wheel encoder ticks per revolution
//   b    if true reverse encoder direction on right side
// notes:
//   right wheel encoder may need to be reversed but for some reason encoder->SetReverseDirection isn't
//   supported in simulation mode (throws exception) and can only be set in the constructor
//   (which is inconvenient when using "GPMotors", since that would require passing in an additional argument)
//   An alternative solution is to just invert the "DistancePerPulse" value
// ===========================================================================================================
void DriveTrain::SetDistancePerPulse(double d, double t, bool b){
	cpr=12.0*t/M_PI/d;
	//std::cout<<"codes per rev:"<<cpr<<std::endl;
	left_motor.ConfigEncoderCodesPerRev(cpr);
	right_motor.ConfigEncoderCodesPerRev(cpr);
	squared_inputs=false;
}
// ===========================================================================================================
// DriveTrain::GetDistancePerPulse()
//  - return Distance per encoder tick (in feet)
// ===========================================================================================================
double DriveTrain::GetDistancePerPulse() {
	return cpr;
}

/**
 * When no other command is running let the operator drive around
 * using the PS3 joystick.
 */
void DriveTrain::InitDefaultCommand() {
	SetDefaultCommand(new TankDriveWithJoystick());
}

/**
 * The log method puts interesting information to the SmartDashboard.
 */
void DriveTrain::Log() {
//	SmartDashboard::PutNumber("Drive Left Distance", left_motor.GetDistance());
//	SmartDashboard::PutNumber("Drive Right Distance", right_motor.GetDistance());
//	SmartDashboard::PutNumber("Drive Left Speed", left_motor.GetVelocity());
	SmartDashboard::PutNumber("Drive Left Speed", GetLeftVoltage());
	SmartDashboard::PutNumber("Drive Right Speed", GetRightVoltage());
	SmartDashboard::PutNumber("Drive Heading", GetHeading());
}

void DriveTrain::Limit(double &num) {
  if (num > 1.0)
    num= 1.0;
  if (num < -1.0)
    num= -1.0;
}
// square the inputs (while preserving the sign) to increase fine control
// while permitting full power
void DriveTrain::SquareInputs(double &left, double &right) {
	if (left >= 0.0)
		left = (left * left);
	else
		left = -(left * left);
	if (right >= 0.0)
		right = (right * right);
	else
		right = -(right * right);
}

void DriveTrain::Drive(Joystick* joy) {
#if JOYTYPE == XBOX_GAMEPAD
	double left=-joy->GetRawAxis(1);
	double right=-joy->GetRawAxis(4);
#else
	double left=-joy->GetY();
	double right=-joy->GetRawAxis(4);
#endif
	if(reverse_driving){
		double r=right;
		double l=left;
		left=-r;
		right=-l;
	}
	//std::cout<<"l:"<<left<<" r:"<<right<<std::endl;
	Limit(left);
	Limit(right);
	if(squared_inputs)
		SquareInputs(left,right);
	left=Deadband(left,x_deadband);
	right=Deadband(right,y_deadband);
	Drive(left,right);
}

double DriveTrain::Deadband(double x, double ignore) {
	return fabs(x)>=ignore ? x: 0.0;
}
void DriveTrain::SetDeadband(double x, double y) {
	x_deadband=x;
	y_deadband=y;
}

void DriveTrain::SetDistance(double d){
	target_distance=d;
	left_motor.SetSetpoint(d);
	right_motor.SetSetpoint(d);
}


void DriveTrain::Turn(double d){
	left_motor.Set(d);
	if(inverted)
		right_motor.Set(d);
	else
		right_motor.Set(-d);
	Log();
}

void DriveTrain::Drive(double l,double r){
	left_motor.Set(l);
	if(inverted)
		right_motor.Set(-r);
	else
		right_motor.Set(r);
	Log();
}
void DriveTrain::Reset() {
	right_motor.Reset();
	left_motor.Reset();
	gyro.Reset();
}
void DriveTrain::Enable() {
	right_motor.Enable();
	left_motor.Enable();
	disabled=false;
}
void DriveTrain::Disable() {
	right_motor.Disable();
	left_motor.Disable();
	disabled=true;
}
void DriveTrain::EndTravel() {
	Drive(0,0);
}
void DriveTrain::DisablePID() {
	right_motor.Disable();
	left_motor.Disable();
	pid_disabled=true;
}
void DriveTrain::EnablePID() {
	right_motor.EnableControl();
	left_motor.EnableControl();
	pid_disabled=false;
}

double DriveTrain::GetHeading() {
	return gyro.GetAngle();
}

void DriveTrain::TeleopInit() {
	std::cout << "DriveTrain::TeleopInit"<<std::endl;
	Reset();
	Enable();
}

void DriveTrain::AutonomousInit() {
	std::cout << "DriveTrain::AutonomousInit"<<std::endl;
	Drive(0,0);
	Reset();
	gyro.Reset();
}

void DriveTrain::DisabledInit() {
	std::cout << "DriveTrain::DisabledInit"<<std::endl;
	Disable();
	Drive(0,0);
	Reset();
}

double DriveTrain::GetDistance() {
	return (left_motor.GetPosition() + right_motor.GetPosition())/2;
}
double DriveTrain::GetSpeed() {
	return (left_motor.GetSpeed() + right_motor.GetSpeed())/2;
}
double DriveTrain::GetLeftSpeed(){
	return left_motor.GetSpeed();
}
double DriveTrain::GetRightSpeed(){
	return right_motor.GetSpeed();
}
double DriveTrain::GetLeftDistance(){
	return left_motor.GetPosition();
}
double DriveTrain::GetRightDistance(){
	return right_motor.GetPosition();
}
double DriveTrain::GetLeftVoltage(){
	return left_motor.Get();
}
double DriveTrain::GetRightVoltage(){
	return right_motor.Get();
}

void DriveTrain::SetReverseDriving(bool b) {
	reverse_driving=b;
}

bool DriveTrain::ReverseDriving() {
	return reverse_driving;
}
