/*
 * ShootBall.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_SHOOTBALL_H_
#define SRC_COMMANDS_SHOOTBALL_H_

#include <Commands/Command.h>

class ShootBall: public Command {
private:
	int state;
	double elapsed_time;
	bool timing=false;
	bool finished=false;

	void OpenGate();
	void TurnFlywheelsOn();
	void PushBall();
	void Reset();

	void SetDeltaTimeout(double t);
	bool Timing() { return timing;}
	bool CheckTimeout();

public:
	ShootBall();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_SHOOTBALL_H_ */
