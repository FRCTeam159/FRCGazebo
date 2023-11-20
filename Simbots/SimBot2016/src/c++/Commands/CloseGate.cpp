/*
 * CloseGate.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#include <Commands/CloseGate.h>
#include "Robot.h"

CloseGate::CloseGate() : Command("CloseGate") {
	Requires(Robot::holder.get());
	std::cout << "new CloseGate"<< std::endl;
}

// Called just before this Command runs the first time
void CloseGate::Initialize() {
	Robot::holder->CloseGate();
	std::cout << "CloseGate started"<<std::endl;
}
bool CloseGate::IsFinished() {
	return Robot::holder->IsGateClosed();
}

void CloseGate::End() {
	std::cout << "CloseGate complete"<<std::endl;
}
