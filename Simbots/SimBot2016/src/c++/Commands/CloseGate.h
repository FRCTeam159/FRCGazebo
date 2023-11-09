/*
 * CloseGate.h
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_CLOSEGATE_H_
#define SRC_COMMANDS_CLOSEGATE_H_

#include <Commands/Command.h>

class CloseGate: public Command {
public:
	CloseGate();
	void Initialize();
	bool IsFinished();
	void Execute() {}
	void End();
	void Interrupted() { End();}
};

#endif /* SRC_COMMANDS_CLOSEGATE_H_ */
