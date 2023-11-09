/*
 * InitShooter.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#include <Commands/ExecShooter.h>
#include "Robot.h"

ExecShooter::ExecShooter() : Command("ExecShooter")  {
	Requires(Robot::shooter.get());
	std::cout << "new ExecShooter()"<< std::endl;
}

void ExecShooter::Initialize() {
	if(!Robot::shooter->IsInitialized())
		Robot::shooter->Initialize();
}

bool ExecShooter::IsFinished() {
	return false;
}

void ExecShooter::Execute() {
	Robot::shooter->Execute();
}

void ExecShooter::End() {
	//std::cout << "ExecShooter::End()"<< std::endl;
}

void ExecShooter::Interrupted() {
	//std::cout << "InitShooter::Interrupted()"<< std::endl;
}
