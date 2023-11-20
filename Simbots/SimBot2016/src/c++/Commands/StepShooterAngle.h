/*
 * StepShooterAngle.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_STEPSHOOTERANGLE_H_
#define SRC_COMMANDS_STEPSHOOTERANGLE_H_

#include <Commands/Command.h>

class StepShooterAngle: public Command {
	double direction;
public:
	StepShooterAngle(double a);
	~StepShooterAngle();
	void Initialize();
	void Execute() {}
	bool IsFinished();
	void End();
	void Interrupted() {End();}
};

#endif /* SRC_COMMANDS_STEPSHOOTERANGLE_H_ */
