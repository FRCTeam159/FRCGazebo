/*
 * ToggleRollers.h
 *
 *  Created on: Mar 27, 2016
 *      Author: dean
 */

#ifndef SRC_COMMANDS_TOGGLEROLLERS_H_
#define SRC_COMMANDS_TOGGLEROLLERS_H_

#include <Commands/Command.h>

class ToggleRollers: public Command {
	int initial_state;
	int target_state;
public:
	ToggleRollers();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted() { End();}
};

#endif /* SRC_COMMANDS_TOGGLEROLLERS_H_ */
