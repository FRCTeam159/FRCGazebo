/*
 * ToggleMode.cpp
 *
 *  Created on: Mar 21, 2016
 *      Author: alpiner
 */

#include <Commands/ToggleMode.h>
#include "Robot.h"

ToggleMode::ToggleMode()  : Command("ToggleMode"){
	//Requires(Robot::oi.get());
	last_state=OI::GetMode();
	std::cout << "new ToggleMode()"<< std::endl;

}
// Called just before this Command runs the first time
void ToggleMode::Initialize() {
	int old_state=OI::GetMode();
	if(old_state==SHOOTING)
		Robot::SetMode(LOADING);
	else
		Robot::SetMode(SHOOTING);
	int new_state=OI::GetMode();
	last_state=new_state;
}
