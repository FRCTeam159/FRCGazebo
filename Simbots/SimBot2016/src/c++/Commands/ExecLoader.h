/*
 * LoaderGotoZero.h
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_EXECLOADER_H_
#define SRC_COMMANDS_EXECLOADER_H_

#include <Commands/Command.h>

class ExecLoader: public Command {
	int state;
	void Low();
	void Load();
	void Idle();

public:
	ExecLoader();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_EXECLOADER_H_ */
