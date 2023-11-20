/*
 * InitShooter.h
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_EXECSHOOTER_H_
#define SRC_COMMANDS_EXECSHOOTER_H_

#include <Commands/Command.h>

class ExecShooter: public Command {
public:
	ExecShooter();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_EXECSHOOTER_H_ */
