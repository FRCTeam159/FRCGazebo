/*
 * DriveStraight.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#include <Commands/DriveStraight.h>
#include "Robot.h"

#define FEET_PER_METER 3.28084
#define MAX_HEADING_ERROR 1 // degrees
#define MAX_POSITION_ERROR 0.1 // meters

#define AP 0.7
#define AI 0.001
#define AD 0.2

#define DP 0.7
#define DI 0.001
#define DD 0.3

#define SCALE 0.8

//#define DEBUG_COMMAND

#define DRIVE_TIMEOUT 1
#define PIDUPDATERATE 0.01

double DriveStraight::distance_error=0;
double DriveStraight::angle_error=0;
double DriveStraight::distance=0;
double DriveStraight::heading=0;

DriveStraight::DriveStraight(double d, double h) : Command("DriveStraight"),
	Acntrl(AP,AI,AD),Dcntrl(DP,DI,DD)
{
	Requires(Robot::drivetrain.get());
    distance=d*FEET_PER_METER;
    heading=h;
    at_position=at_heading=false;
	std::cout << "new DriveStraight("<<distance<<","<<heading<<")"<< std::endl;
}
// Called just before this Command runs the first time
void DriveStraight::Initialize() {
	SetTimeout(DRIVE_TIMEOUT*distance+1);
	Acntrl.Initialize(heading);
	Dcntrl.Initialize(distance);
    distance_error=angle_error=0;

	//Robot::drivetrain->DriveStraight(distance*FEET_PER_METER);
	std::cout << "DriveStraight Started .."<<std::endl;
}
void DriveStraight::Execute() {
//	double l=Robot::drivetrain->GetLeftDistance();
//	double r=Robot::drivetrain->GetRightDistance();
//	double h=Robot::drivetrain->GetHeading();
//	std::cout << "DriveStraight "<<l<<","<<r<<","<<h<<")"<<std::endl;
}

bool DriveStraight::IsFinished() {
	if(IsTimedOut()){
		std::cout << "DriveStraight Error:  Timeout expired"<<std::endl;
		return true;
	}
	at_heading=Acntrl.AtTarget();
	at_position=Dcntrl.AtTarget();
	//return(at_position && at_heading);
    return(at_position);

}

void DriveStraight::End() {
	double l=Robot::drivetrain->GetLeftDistance();
	double r=Robot::drivetrain->GetRightDistance();
	double h=Robot::drivetrain->GetHeading();
	std::cout << TimeSinceInitialized()<< "  DriveStraight End("<<l<<","<<r<<","<<h<<")"<<std::endl;
	Acntrl.End();
	Dcntrl.End();
	Robot::drivetrain->EndTravel();
}
void DriveStraight::Interrupted() {
	std::cout << "DriveStraight::Interrupted"<<std::endl;
	End();
}

// =============== Inner Class AngleControl =============================
DriveStraight::AngleControl::AngleControl(double P, double I, double D):
	pid(P,I,D,this,this,PIDUPDATERATE)
{
	target=0;
}

double DriveStraight::AngleControl::PIDGet()
{
	return Robot::drivetrain->GetHeading();
}

// ================================================================================
// DriveStraight::AngleControl::PIDWrite
// ================================================================================
//  - Collect correction values from both Distance and Angle PIDs
//  - Adjust speed on left and right motors to turn slightly to correct angle error
//    o Subtract angle correction from current speed of left side
//    o Add angle correction from current speed of left side
//  - If either correction > 1 (max motor input) scale the result
//  - Apply correction using drive-train tank drive function: Drive(left,right)
// ================================================================================
void DriveStraight::AngleControl::PIDWrite(double a)
{
	DriveStraight::angle_error=a;
	double d=DriveStraight::distance_error;
	double m1=d+a;
	double m2=d-a;
	double mx=m1>m2?m1:m2;
	double scale=mx>1?1/mx:1;
	double l=m2*scale;
	double r=m1*scale;
#ifdef DEBUG_COMMAND
	double s=Robot::drivetrain->GetDistance();
    double h=Robot::drivetrain->GetHeading();
	std::cout << "heading:"<<h<<" distance:"<<s
            <<" err:"<<h-heading<<" "<<s-distance
	        <<" corr:"<<a<<" "<<d
	        <<std::endl;
#endif

	Robot::drivetrain->Drive(l,r);
	//Robot::drivetrain->Turn(-d);
}
void DriveStraight::AngleControl::Initialize(double d)
{
	target=d;
	pid.Reset();
	pid.SetAbsoluteTolerance(MAX_HEADING_ERROR);
	//pid.SetInputRange(-180,180);
	pid.SetSetpoint(target);
	pid.Enable();
}
bool DriveStraight::AngleControl::AtTarget(){
	return pid.OnTarget();
}
void DriveStraight::AngleControl::End(){
	pid.Disable();
}

// =============== Inner Class DistanceControl =============================
DriveStraight::DistanceControl::DistanceControl(double P, double I, double D):
	pid(P,I,D,this,this,PIDUPDATERATE)
{
	target=0;
}

double DriveStraight::DistanceControl::PIDGet()
{
	return Robot::drivetrain->GetDistance();
}
// ================================================================================
// DriveStraight::DistanceControl::PIDWrite
// ================================================================================
// - capture the Distance error from the PID and save it to a static variable
// - Let the AngleControl PID do the actual correction
// ================================================================================
void DriveStraight::DistanceControl::PIDWrite(double d)
{
	DriveStraight::distance_error=d;
}
void DriveStraight::DistanceControl::Initialize(double d)
{
	target=d;
	Robot::drivetrain->Reset();
	pid.Reset();
	pid.SetAbsoluteTolerance(MAX_POSITION_ERROR);
	pid.SetSetpoint(target);
	pid.Enable();
}
bool DriveStraight::DistanceControl::AtTarget(){
	double d=Robot::drivetrain->GetDistance();
	return (d>=target)?true:false;
	//return pid.OnTarget();
}
void DriveStraight::DistanceControl::End(){
	pid.Disable();
}
