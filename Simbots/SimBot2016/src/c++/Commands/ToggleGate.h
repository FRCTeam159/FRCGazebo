/*
 * OpenGate.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_TOGGLEGATE_H_
#define SRC_COMMANDS_TOGGLEGATE_H_

#include <Commands/Command.h>

class ToggleGate: public Command {
	enum {
		UNDEFINED = 0,
		CLOSED = 1,
		OPEN =2
	};
	int current_state;
	int target_state;
public:
	ToggleGate();
	void Initialize();
	void Execute() {}
	bool IsFinished();
	void End();
	void Interrupted() { End();}
};

#endif /* SRC_COMMANDS_TOGGLEGATE_H_ */
