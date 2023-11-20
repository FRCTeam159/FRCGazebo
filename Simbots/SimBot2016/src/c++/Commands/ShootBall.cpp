/*
 * ShootBall.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/ShootBall.h>
#include "Robot.h"
#define GATE_DELAY 1
#define FLYWHEEL_DELAY 1
#define PUSH_DELAY 1
#define RESET_DELAY 0.1

enum {
	OPEN_GATE,
	TURN_FLYWHEELS_ON,
	PUSH_BALL,
	RESET,
};


ShootBall::ShootBall() : Command("ShootBall") {
	Requires(Robot::shooter.get());
	Requires(Robot::holder.get());
	state=0;
	elapsed_time=0;
	std::cout << "new ShootBall"<< std::endl;
}

//*********** Utility Functions ******************************************************

#define DEBUG_COMMAND
#ifdef DEBUG_COMMAND
#define DEBUG_PRINT(s) \
	std::cout<<TimeSinceInitialized()<<" ShootBall: " << s <<std::endl
#else
  #define DEBUG_PRINT(s)
#endif

void ShootBall::SetDeltaTimeout(double t) {
	timing=true;
	elapsed_time=TimeSinceInitialized();
	SetTimeout(elapsed_time+t);
}

bool ShootBall::CheckTimeout() {
	if(IsTimedOut()){
		timing=false;
		return true;
	}
	return false;
}
//*********** Command Override Functions **************************************************

// Called just before this Command runs the first time
void ShootBall::Initialize() {
	elapsed_time=0;
	finished=false;
	DEBUG_PRINT("Initialize");
	if(Robot::holder->IsBallPresent()){
		if(Robot::holder->IsGateClosed())
			state=OPEN_GATE; // In teleop mode Holder state machine should take care of this
		else
			state=TURN_FLYWHEELS_ON;
	}
	else{
		std::cout << "Shoot Aborted (no ball detected)"<<std::endl;
		finished=true;
	}
}

// Called repeatedly while this Command is running
void ShootBall::Execute() {
	switch(state){
	case OPEN_GATE:
		OpenGate();
		break;
	case TURN_FLYWHEELS_ON:
		TurnFlywheelsOn();
		break;
	case PUSH_BALL:
		PushBall();
		break;
	case RESET:
		Reset();
	}
}
bool ShootBall::IsFinished() {
	return finished;
}
void ShootBall::End() {
	DEBUG_PRINT("End");
	Robot::shooter->DisableFlywheels();
	Robot::holder->SetPushHoldSpeed(0.0);
}
void ShootBall::Interrupted() {
	DEBUG_PRINT("Interrupted");
	Robot::shooter->DisableFlywheels();
	Robot::holder->SetPushHoldSpeed(0.0);
}

//*********** State Machine Functions ******************************************************
//==========================================================================================
// ShootBall::OpenGate()
//==========================================================================================
// - Pinch the Ball
//==========================================================================================
void ShootBall::OpenGate(){
	Robot::holder->OpenGate();
	if(Robot::holder->IsGateOpen()){
		DEBUG_PRINT("Gate Open ");
		state=TURN_FLYWHEELS_ON;
	}

}
//==========================================================================================
// ShootBall::TurnFlywheelsOn()
//==========================================================================================
// - Turn on the flywheels
// - Wait for a minimum delay
// - Wait until flywheels are at target speed
// - Then Push the ball
//==========================================================================================
void ShootBall::TurnFlywheelsOn(){
	Robot::holder->HoldBall();
	if(!Timing()){
		DEBUG_PRINT("Enabling Flywheels ..");
		Robot::shooter->EnableFlywheels();
		SetDeltaTimeout(FLYWHEEL_DELAY);
	}
	if(Robot::shooter->IsAtSpeed()){
			DEBUG_PRINT("Flywheels at speed, PushBall started ..");
		state=PUSH_BALL;
	}
	else if(CheckTimeout()){
		DEBUG_PRINT("Flywheels NOT not speed, shooting anyway ..");
		state=PUSH_BALL;
	}
}

//==========================================================================================
// ShootBall::PushBall()
//==========================================================================================
// - Send a Push request to the Holder
// - Wait a minimum period to see if the ball has left
// - If the ball is still present assume the shot failed and the ball is stuck and exit
//   Assume the holder state machine will try to eject the ball
// - Otherwise, assume the shot succeeded and reset the shooter
//==========================================================================================
void ShootBall::PushBall(){
	Robot::holder->PushBall();
	if(!Timing())
		SetDeltaTimeout(PUSH_DELAY);
	else if(CheckTimeout()){
		if(Robot::holder->IsBallPresent()){ // Error
			DEBUG_PRINT("Shoot Error (ball did not leave before timeout) - Exiting");
			finished=true;
		}
		else
			state=RESET;
	}
}

//==========================================================================================
// ShootBall::Reset()
//==========================================================================================
// - Reset the shooter and holder
//==========================================================================================
void ShootBall::Reset(){
	DEBUG_PRINT("Shot Complete - Resetting Subsystems");
	//double min=Robot::shooter->GetMinAngle();
	//Robot::shooter->SetTargetAngle(min);
	Robot::shooter->Reset();
	Robot::holder->Reset();
    Robot::vision->Reset();

	finished=true;
}


