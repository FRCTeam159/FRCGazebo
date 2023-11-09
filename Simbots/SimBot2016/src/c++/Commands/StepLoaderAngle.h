/*
 * StepLoaderAngle.h
 *
 *  Created on: Mar 21, 2016
 *      Author: alpiner
 */

#ifndef SRC_STEPLOADERANGLE_H_
#define SRC_STEPLOADERANGLE_H_

#include <Commands/Command.h>

class StepLoaderAngle: public Command {
	double direction;
public:
	StepLoaderAngle(double a);
	void Initialize();
	void Execute() {}
	bool IsFinished();
	void End();
	void Interrupted() {End();}
};

#endif /* SRC_STEPLOADERANGLE_H_ */
