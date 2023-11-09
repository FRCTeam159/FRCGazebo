/*
 * LoaderGotoZero.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#include <Commands/ExecLoader.h>
#include "Robot.h"

enum {
	LOAD,
	LOW,
	IDLE
};

ExecLoader::ExecLoader() : Command("ExecLoader") {
	Requires(Robot::loader.get());
	// note: enabling this line causes shooter angle adjust etc. to stop working (why ??)
	// Requires(Robot::holder.get());
	std::cout << "new ExecLoader()"<< std::endl;
	state=IDLE;
}

//*********** Private Utility Functions ***************************************************

#define DEBUG_COMMAND

#ifdef DEBUG_COMMAND
#define DEBUG_PRINT(s) \
	std::cout<<TimeSinceInitialized()<<" ExecLoader: " << s <<std::endl
#else
  #define DEBUG_PRINT(s)
#endif

//*********** Command Override Functions **************************************************

void ExecLoader::Initialize() {
	if(!Robot::loader->IsInitialized())
		Robot::loader->Initialize();
}

bool ExecLoader::IsFinished() {
	return true;
}

void ExecLoader::Execute() {
	Robot::loader->Execute();
	switch(state){
	case LOW:
		Low();
		break;
	case LOAD:
		Load();
		break;
	case IDLE:
		Idle();
		break;
	}
}

void ExecLoader::Interrupted() {
	//DEBUG_PRINT("Interrupted");
}
void ExecLoader::End() {
	//DEBUG_PRINT("End");
}

//*********** State Machine Functions ******************************************************

void ExecLoader::Low() {
	if(Robot::loader->Loading() && !Robot::holder->IsBallPresent()){
		DEBUG_PRINT("LOW: Loading mode Started");
		state=LOAD;
	}
	else if(Robot::loader->LifterAtLowerLimit()){
		DEBUG_PRINT("Lifter at Lower limit");
		state=IDLE;
	}
	else
		Robot::loader->GoToZeroLimitSwitch();
}

void ExecLoader::Load() {
	if(!Robot::loader->Loading()){
		DEBUG_PRINT("Load Cancelled");
		state=LOW;
	}
	else if(Robot::holder->IsBallPresent()){
		DEBUG_PRINT("Ball Loaded - Resetting lifter");
		state=LOW;
	}
	else
		Robot::loader->ExecLoad();
}
void ExecLoader::Idle() {
	if(Robot::loader->Loading() && !Robot::holder->IsBallPresent()){
		DEBUG_PRINT("IDLE: Loading mode Started");
		state=LOAD;
	}
}

