/*
 * FullStop.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: dean
 */

#include <Commands/FullStop.h>
#include "Robot.h"

FullStop::FullStop() {
	Requires(Robot::drivetrain.get());
}

void FullStop::Initialize() {
	std::cout << "FullStop started .."<<std::endl;
	Robot::drivetrain->Disable();
}
bool FullStop::IsFinished() {
	return Robot::drivetrain->IsDisabled();
}
void FullStop::End() {
	std::cout << "FullStop complete"<<std::endl;
}
