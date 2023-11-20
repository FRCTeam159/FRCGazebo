/*
 * OpenGate.h
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_OPENGATE_H_
#define SRC_COMMANDS_OPENGATE_H_

#include <Commands/Command.h>

class OpenGate: public Command {
public:
	OpenGate();
	void Initialize();
	bool IsFinished();
	void Execute() {}
	void End();
	void Interrupted() { End();}
};

#endif /* SRC_COMMANDS_OPENGATE_H_ */
