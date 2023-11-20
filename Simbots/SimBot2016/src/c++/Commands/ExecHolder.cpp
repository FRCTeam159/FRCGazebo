/*
 * InitHolder.cpp
 *
 *  Created on: Mar 29, 2016
 *      Author: alpiner
 *
 *  Default Command for Holder SubSystem
 *  Called continuously in Teleop mode when no other Command is running
 */

#include <Commands/ExecHolder.h>
#include "Robot.h"

#define BALLDETECTIONDELAY 0.5
#define BALLREMOVEDELAY 2.0

enum {
	FIND_ZERO,
	WAIT_FOR_BALL_TO_ENTER,
	GO_TO_FORWARD_LIMIT,
	WAIT_FOR_PUSH_REQUEST,
	WAIT_FOR_BALL_TO_LEAVE,
	GO_TO_REVERSE_LIMIT,
	REMOVE_BALL,
	PUSH_ERROR
};

#define DEBUG_COMMAND

#ifdef DEBUG_COMMAND
#define DEBUG_PRINT(s) \
	std::cout<<TimeSinceInitialized()<<" ExecHolder: " << s <<std::endl
#else
  #define DEBUG_PRINT(s)
#endif

ExecHolder::ExecHolder() : Command("ExecHolder")  {
	Requires(Robot::holder.get());
	std::cout << "new ExecHolder()"<< std::endl;
	state=FIND_ZERO;
	elapsed_time=0;
}

//*********** Private Utility Functions ***************************************************

void ExecHolder::SetDeltaTimeout(double t) {
	elapsed_time=TimeSinceInitialized();
	timing=true;
	SetTimeout(elapsed_time+t);
}

bool ExecHolder::CheckTimeout() {
	if(IsTimedOut()){
		timing=false;
		return true;
	}
	return false;
}

//*********** Command Override Functions **************************************************

void ExecHolder::Initialize() {
	if(!Robot::holder->IsInitialized()){
		DEBUG_PRINT("Initializing Holder");
		Robot::holder->Initialize();
	}
}

bool ExecHolder::IsFinished() {
	return false; // run unless interrupted
}

void ExecHolder::End() {
	DEBUG_PRINT("End");
}

void ExecHolder::FindZero() {
	if(Robot::holder->FindZero()){
		DEBUG_PRINT("Zero Limit switch found");
		state=WAIT_FOR_BALL_TO_ENTER;
	}
}

void ExecHolder::Interrupted() {
	DEBUG_PRINT("Interrupted");
}

//==========================================================================================
// ExecHolder::Execute()
//==========================================================================================
// - Command function override
// - Holder "Autohold" state machine
//==========================================================================================
void ExecHolder::Execute() {
	Robot::holder->Execute();
	switch(state){
	case FIND_ZERO:
		FindZero();
		break;
	case WAIT_FOR_BALL_TO_ENTER:
		WaitForBallToEnter();
		break;
	case GO_TO_FORWARD_LIMIT:
		GoToForwardLimit();
		break;
	case WAIT_FOR_PUSH_REQUEST:
		WaitForPushRequest();
		break;
	case WAIT_FOR_BALL_TO_LEAVE:
		WaitForBallToLeave();
		break;
	case GO_TO_REVERSE_LIMIT:
		GoToReverseLimit();
		break;
	case REMOVE_BALL:
		RemoveBall();
		break;
	case PUSH_ERROR:
		PushError();
		break;
	}
}

//*********** State Machine Functions ******************************************************

//==========================================================================================
// ExecHolder::WaitForBallToEnter()
//==========================================================================================
// - Wait for ball to enter
// - Then delay to give the ball time to settle
// - Then pinch the ball (goto forward limit)
//==========================================================================================
void ExecHolder::WaitForBallToEnter() {
	//Robot::holder->CloseGate();
	if(Robot::holder->IsBallPresent()){
		if(!Timing()){
			SetDeltaTimeout(BALLDETECTIONDELAY);
			DEBUG_PRINT("New Ball Detected - Waiting Settling Period..");
		}
		else if(CheckTimeout()){
			DEBUG_PRINT("Opening Gate ..");
			state=GO_TO_FORWARD_LIMIT;
		}
	}
	else
		Robot::holder->CloseGate();
}

//==========================================================================================
// ExecHolder::GoToForwardLimit()
//==========================================================================================
// - Pinch the ball
//==========================================================================================
void ExecHolder::GoToForwardLimit() {
	if(!Robot::holder->IsBallPresent()){
		state=GO_TO_REVERSE_LIMIT;
	}
	else{
		Robot::holder->OpenGate();
		if(Robot::holder->IsGateOpen()){
			state=WAIT_FOR_PUSH_REQUEST;
		}
	}
}

//==========================================================================================
// ExecHolder::GoToForwardLimit()
//==========================================================================================
// - Hold the ball while waiting for a push request
//==========================================================================================
void ExecHolder::WaitForPushRequest() {
	if(!Robot::holder->IsBallPresent()){
		DEBUG_PRINT("Error - Ball not present");
		state=WAIT_FOR_BALL_TO_ENTER;
	}
	else{
		Robot::holder->HoldBall();
		if(Robot::holder->PushRequested()){
			DEBUG_PRINT("Push Requested - Waiting for ball to leave..");
			state=WAIT_FOR_BALL_TO_LEAVE;
		}
	}
}

//==========================================================================================
// ExecHolder::WaitForBallToLeave()
//==========================================================================================
// - After a push request, enable the push wheel (forward push)
// - Start a timer to detect when the ball leaves
// - If the ball is still present after the timeout, assume it is stuck and try to remove it
// - Else close the gate and wait for a new ball to enter
//==========================================================================================
void ExecHolder::WaitForBallToLeave() {
	Robot::holder->PushBall();
	if(!Timing())
		SetDeltaTimeout(BALLDETECTIONDELAY);
	else if(CheckTimeout()){
		if(Robot::holder->IsBallPresent()){
			DEBUG_PRINT("Push Failed - Attempting to remove the ball");
			state=REMOVE_BALL;
		}
		else{
			DEBUG_PRINT("Push Succeeded - Closing Gate");
			state=GO_TO_REVERSE_LIMIT;
		}
	}
}

//==========================================================================================
// ExecHolder::GoToReverseLimit()
//==========================================================================================
// - Move the gate backwards until it closes
//==========================================================================================
void ExecHolder::GoToReverseLimit() {
	Robot::holder->CloseGate();
	if(Robot::holder->IsGateClosed()){
		DEBUG_PRINT("Gate Closed - Waiting for ball to enter");
		state=WAIT_FOR_BALL_TO_ENTER;
	}
}

//==========================================================================================
// ExecHolder::RemoveBall()
//==========================================================================================
// - Try to remove a stuck ball by reversing the push and gate motors
// - Start a timer
// - If the timeout expires and the ball is still stuck go to an error state
// - Else close the gate
//==========================================================================================
void ExecHolder::RemoveBall() {
	Robot::holder->RemoveBall();
	if(!Timing())
		SetDeltaTimeout(BALLDETECTIONDELAY);
	else if(CheckTimeout()){
		if(Robot::holder->IsBallPresent()){
			DEBUG_PRINT("Remove Ball Failed - Entering Error State");
			state=PUSH_ERROR;
		}
		else {
			DEBUG_PRINT("Remove Ball Succeeded - Closing Gate");
			state=GO_TO_REVERSE_LIMIT;
		}
	}
}

//==========================================================================================
// ExecHolder::PushError()
//==========================================================================================
// - The ball is stuck
// - Keep trying to close the gate
// - Wait until the ball gets removed somehow
//==========================================================================================
void ExecHolder::PushError() {
	Robot::holder->CloseGate();
	if(!Robot::holder->IsBallPresent() && Robot::holder->IsGateClosed())
		state = WAIT_FOR_BALL_TO_ENTER;
}


