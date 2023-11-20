/*
 * Shooter.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */
#include <Commands/ExecShooter.h>
#include <Subsystems/Shooter.h>
#include <WPILib.h>
#include "Assignments.h"

#define FWSPEED 75
#define FP 0.002
#define FI 0.00001
#define FD 0.0001

#define AP 0.02
#define AI 0.001
#define AD 0.1

#define AMIN 0
#define AMAX 70
#define MAX_SPEED_ERROR 10
#define MAX_ANGLE_ERROR 2
#define GOTO_LOWER_SPEED -0.6
#define PID_UPDATE_PERIOD 0.01

//#define DEBUG_ANGLE_PID

#ifdef SIMULATION
#define ENCODER_TICKS 360
#else
#define ENCODER_TICKS 900
#endif
Shooter::Shooter() : Subsystem("Shooter"),
	angleMotor(SHOOTER_ANGLE),
	leftFWMotor(SHOOTER_LEFT),
	rightFWMotor(SHOOTER_RIGHT),
	accel(SHOOTER_PITCH)
{
	std::cout<<"New Shooter("<<SHOOTER_ANGLE<<","<<SHOOTER_LEFT<<","<<SHOOTER_RIGHT<<")"<<std::endl;
	max_angle=AMAX; // max elevation (degrees)
	min_angle=AMIN;

	angleMotor.ConfigLimitMode(CANTalon::kLimitMode_SwitchInputsOnly);

	leftFWMotor.SetFeedbackDevice(CANTalon::QuadEncoder);
	rightFWMotor.SetFeedbackDevice(CANTalon::QuadEncoder);
	leftFWMotor.SetControlMode(CANTalon::kSpeed);
	rightFWMotor.SetControlMode(CANTalon::kSpeed);

	leftFWMotor.SetPID(FP, FI, FD);
	leftFWMotor.ConfigEncoderCodesPerRev(ENCODER_TICKS);
	//leftFWMotor.SetDebug(1);

	rightFWMotor.SetPID(FP, FI, FD);
	rightFWMotor.ConfigEncoderCodesPerRev(ENCODER_TICKS);

	flywheel_target=FWSPEED;
	flywheel_speed=0;
	max_angle_error=MAX_ANGLE_ERROR;
	angle=0;
	angle_pid=new PIDController(AP, AI, AD,this,this,PID_UPDATE_PERIOD);
	angle_pid->Reset(); // clear IAccum
	angle_pid->SetSetpoint(0);
	angle_pid->SetAbsoluteTolerance(max_angle_error);
	//angle_pid->SetToleranceBuffer(4);
	initialized=false;
	Log();
}

Shooter::~Shooter(){
	if(angle_pid)
		delete angle_pid;
}
void Shooter::Log() {
	SmartDashboard::PutBoolean("Shooter Initialized", IsInitialized());

	LogAngle(GetAngle());
	LogSpeed(GetFWSpeed());
}

void Shooter::InitDefaultCommand() {
	SetDefaultCommand(new ExecShooter());
}

void Shooter::Execute() {
	Log();
	if(!initialized){
		angleMotor.Set(GOTO_LOWER_SPEED);
		if(AtLowerLimit())
			SetInitialized();
	}
}

double Shooter::PIDGet() {
	return accel.GetAngle();
}

void Shooter::PIDWrite(double output){
#ifdef DEBUG_ANGLE_PID
	if(!angle_pid->OnTarget())
	std::cout<<"Shooter::PIDWrite target:"
			<<angle_pid->GetSetpoint()
			<<" error:"<<angle_pid->GetError()
			<<" correction:"<<output
			<<std::endl;
#endif
	angleMotor.PIDWrite(output);
}
void Shooter::AutonomousInit(){
	std::cout << "Shooter::AutonomousInit"<<std::endl;
	Init();
}
void Shooter::TeleopInit(){
	std::cout << "Shooter::TeleopInit"<<std::endl;
	Init();
}
void Shooter::DisabledInit(){
	std::cout << "Shooter::DisabledInit"<<std::endl;
	Disable();
}

void Shooter::LogSpeed(double d) {
	SmartDashboard::PutNumber("Shooter FW Speed", d);
}
void Shooter::LogAngle(double d) {
	SmartDashboard::PutNumber("Shooter Angle", d);
}

// Initialize
void Shooter::Init(){
	initialized=false;
	angle_pid->Reset();
	angleMotor.Disable();
	leftFWMotor.Reset();
	rightFWMotor.Reset();
	Log();
}

void Shooter::Reset(){
	Disable();
}

void Shooter::Disable(){
	angle_pid->Reset();
	angleMotor.Disable();
	leftFWMotor.Disable();
	rightFWMotor.Disable();

	angleMotor.Disable();
	leftFWMotor.Disable();
	rightFWMotor.Disable();

	angle=0;
	initialized=false;
	Log();
}
// Set the shooter angle
void Shooter::SetTargetAngle(double a){
	a=a>max_angle?max_angle:a;
	a=a<min_angle?min_angle:a;
	angle=a;
	angle_pid->SetSetpoint(angle);
    angle_pid->SetAbsoluteTolerance(max_angle_error);
}

// Set the shooter angle
void Shooter::SetTargetSpeed(double a){
	flywheel_target=a;
}

bool Shooter::IsAtAngle(){
	bool ontarget=angle_pid->OnTarget();
	GetAngle();
	return ontarget;
}
bool Shooter::IsAtSpeed(){
	double ave_speed=GetFWSpeed();
	bool ontarget=fabs(ave_speed-flywheel_target)<=MAX_SPEED_ERROR;
	return ontarget;
}

double Shooter::GetTargetAngle(){
	return angle;
}
double Shooter::GetTargetSpeed(){
	return flywheel_target;
}
double Shooter::GetFWSpeed(){
	double ave_speed=0.5*(leftFWMotor.GetSpeed()+rightFWMotor.GetSpeed());
	LogSpeed(ave_speed);
	return ave_speed;
}
double Shooter::GetAngle(){
	double d=accel.GetAngle();
	LogAngle(d);
	return d;
}

void Shooter::EnableFlywheels(){
	leftFWMotor.Reset();
	rightFWMotor.Reset();
	leftFWMotor.SetSetpoint(flywheel_target);
	rightFWMotor.SetSetpoint(flywheel_target);
	leftFWMotor.Enable();
	rightFWMotor.Enable();
	Log();
}
void Shooter::DisableFlywheels(){
	leftFWMotor.Disable();
	rightFWMotor.Disable();
	leftFWMotor.Reset();
	rightFWMotor.Reset();
	Log();
}

void Shooter::SetInitialized() {
	initialized=true;
	angleMotor.SetSetpoint(0);
	angle=0;
	accel.Reset();
	angleMotor.EnableControl();
}

void Shooter::Initialize() {
	if(!AtLowerLimit()){
		angleMotor.Disable();
		GoToLowerLimitSwitch();
	}
}
bool Shooter::IsInitialized() {
	return initialized;
}

void Shooter::GoToLowerLimitSwitch() {
	angleMotor.Set(GOTO_LOWER_SPEED);
}

bool Shooter::AtLowerLimit() {
	return angleMotor.IsRevLimitSwitchClosed();
}

void Shooter::SetMaxAngleError(double d) {
    max_angle_error=d;
}

void Shooter::EnableAngleController(bool b) {
    if(b)
        angle_pid->Enable();
    else
        angle_pid->Disable();
}

void Shooter::SetAngle(double output) {
    angleMotor.PIDWrite(output);
}

void Shooter::SetDefaultMaxAngleError() {
    max_angle_error=MAX_ANGLE_ERROR;
}

