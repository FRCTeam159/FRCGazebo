/*
 * InitHolder.h
 *
 *  Created on: Mar 29, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_EXECHOLDER_H_
#define SRC_COMMANDS_EXECHOLDER_H_

#include <Commands/Command.h>

class ExecHolder: public Command {
private:
	int state;
	double elapsed_time;
	bool timing=false;
	void SetDeltaTimeout(double t);
	bool CheckTimeout();
	bool Timing() { return timing;}

	void FindZero();
	void WaitForBallToEnter();
	void GoToForwardLimit();
	void WaitForPushRequest();
	void WaitForBallToLeave();
	void GoToReverseLimit();
	void RemoveBall();
	void PushError();

public:
	ExecHolder();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_EXECHOLDER_H_ */
