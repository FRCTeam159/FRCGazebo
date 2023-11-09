/*
 * OpenGate.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#include <Commands/OpenGate.h>
#include "Robot.h"
OpenGate::OpenGate() : Command("OpenGate")  {
	Requires(Robot::holder.get());
	std::cout << "new OpenGate"<< std::endl;
}

// Called just before this Command runs the first time
void OpenGate::Initialize() {
	SetTimeout(1.0);
	Robot::holder->OpenGate();
	std::cout << "OpenGate started .."<<std::endl;
}
bool OpenGate::IsFinished() {
	if(IsTimedOut() || Robot::holder->IsGateOpen()){
		return true;
		//std::cout << "OpenGate complete"<< std::endl;
	}
	return false;
}
void OpenGate::End() {
	std::cout << "OpenGate End"<<std::endl;
}
