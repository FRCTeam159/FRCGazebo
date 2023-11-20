/*
 * StepShooterAngle.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/StepShooterAngle.h>
#include "Robot.h"

#define STEP_TIMEOUT 2

StepShooterAngle::StepShooterAngle(double a) : Command("StepShooterAngle") {
	Requires(Robot::shooter.get());
	Requires(Robot::holder.get());
	std::cout << "new StepShooterAngle("<<a<<")"<< std::endl;
	direction=a;
}
StepShooterAngle::~StepShooterAngle(){
	std::cout << "Deleting StepShooterAngle"<< std::endl;
}
// Called just before this Command runs the first time
void StepShooterAngle::Initialize() {
	SetTimeout(STEP_TIMEOUT);
	double current=Robot::shooter->GetTargetAngle();
	double max=Robot::shooter->GetMaxAngle();
	double min=Robot::shooter->GetMinAngle();

	double target=current+direction;
	target=target>=max?max:target;
	target=target<=min?min:target;
	double push_speed=0.2*target/max;

	std::cout << "Changing Shooter Angle - current:"<< current <<" new:"<<target<<" push:"<<push_speed<<std::endl;
	Robot::holder->SetPushHoldSpeed(push_speed);
	Robot::shooter->SetTargetAngle(target);
	Robot::shooter->EnableAngleController(true);
}

// Make this return true when this Command no longer needs to run execute()
bool StepShooterAngle::IsFinished() {
	if(IsTimedOut()){
		std::cout << "StepAngle Error:  Timeout expired"<<std::endl;
		return true;
	}
	bool ontarget=Robot::shooter->IsAtAngle();
	if(ontarget)
		std::cout << "Shooter On Target:"<<std::endl;
	return ontarget;
}
// Called once after isFinished returns true
void StepShooterAngle::End() {
	//Robot::shooter->DisablePID();
}
