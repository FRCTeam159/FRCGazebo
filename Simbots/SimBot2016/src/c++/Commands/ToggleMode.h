/*
 * ToggleMode.h
 *
 *  Created on: Mar 21, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_TOGGLEMODE_H_
#define SRC_COMMANDS_TOGGLEMODE_H_

#include <Commands/Command.h>

class ToggleMode: public Command {
	int last_state;
public:
	ToggleMode();
	void Initialize();
	void Execute(){}
	bool IsFinished() { return true;}
	void End() {}
	void Interrupted() {End();}
};

#endif /* SRC_COMMANDS_TOGGLEMODE_H_ */
