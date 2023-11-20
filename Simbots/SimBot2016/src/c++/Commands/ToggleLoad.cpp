/*
 * ToggleLoad.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: alpiner
 */

#include <Commands/ToggleLoad.h>
#include "Robot.h"


ToggleLoad::ToggleLoad() : Command("ToggleLoad") {
	Requires(Robot::loader.get());
	Requires(Robot::holder.get());
	std::cout << "new ToggleLoad()"<< std::endl;
}

// ===========================================================================================================
// LoadBall::Initialize()
// - Toggle between Loading and not loading modes
// ===========================================================================================================
void ToggleLoad::Initialize() {
	if(Robot::loader->Loading())
		Robot::loader->SetLow();
	else if(!Robot::holder->IsBallPresent())
		Robot::loader->LoadBall();
}

bool ToggleLoad::IsFinished() {
	return true;
}

