/*
 * ToggleRollers.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: dean
 */

#include <Commands/ToggleRollers.h>
#include "Robot.h"

ToggleRollers::ToggleRollers()  : Command("ToggleRollers") {
	Requires(Robot::loader.get());
}

void ToggleRollers::Initialize() {
	initial_state=Robot::loader->GetRollorState();
	switch(initial_state){
	case ROLLERS_OFF:
		Robot::loader->SetRollerState(ROLLERS_FORWARD);
		break;
	case ROLLERS_FORWARD:
		Robot::loader->SetRollerState(ROLLERS_REVERSE);
		break;
	case ROLLERS_REVERSE:
		Robot::loader->SetRollerState(ROLLERS_OFF);
		break;
	}
}
bool ToggleRollers::IsFinished() {
	return true;
}
void ToggleRollers::End() {
}

void ToggleRollers::Execute() {
}
