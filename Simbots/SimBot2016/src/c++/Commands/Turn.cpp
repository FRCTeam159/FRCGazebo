/*
 * Turn.cpp
 *
 *  Created on: Mar 13, 2016
 *      Author: dean
 */

#include <Commands/Turn.h>
#include "Robot.h"

#define MAX_HEADING_ERROR 2 // degrees
#define TURN_TIMEOUT 3 // seconds

#define AP 0.04
#define AI 0.0005
#define AD 0.05

//#define DEBUG_COMMAND
#define PIDUPDATERATE 0.01

TurnForTime::TurnForTime(double a)  : Command("Turn"), pid(AP,AI,AD,this,this,PIDUPDATERATE)
{
	target=a;
	Requires(Robot::drivetrain.get());
	std::cout << "new Turn("<<target<<")"<< std::endl;
}

void TurnForTime::Initialize() {
	SetTimeout(TURN_TIMEOUT);
	std::cout << "Turn Started .."<<std::endl;
	pid.Reset();
	pid.SetSetpoint(target);
	pid.SetAbsoluteTolerance(MAX_HEADING_ERROR);
	//pid.SetToleranceBuffer(2);
	pid.Enable();
}
bool TurnForTime::IsFinished() {
	if(IsTimedOut()){
		std::cout << "Turn Error:  Timeout expired"<<std::endl;
		return true;
	}
	if( pid.OnTarget()){
		double d=Robot::drivetrain->GetHeading();
#ifdef DEBUG_COMMAND
		std::cout<< "Turn OnTarget target="<<target<<" angle="<<d<<std::endl;
#endif
		return true;
	}
	return false;
}

void TurnForTime::End() {
	double h=Robot::drivetrain->GetHeading();
	std::cout << TimeSinceInitialized()<< "  Turn End("<<h<<")"<<std::endl;
	pid.Disable();
	Robot::drivetrain->EndTravel();
}

double TurnForTime::PIDGet()
{
	double d=Robot::drivetrain->GetHeading();
#ifdef DEBUG_COMMAND
	std::cout << "Turn::PIDGet("<<d<<")"<<std::endl;
#endif
	return d;
}
void TurnForTime::PIDWrite(double d)
{
#ifdef DEBUG_COMMAND
	std::cout << "Turn::PIDWrite("<<d<<")"<<std::endl;
#endif
	Robot::drivetrain->Turn(-d);
}
