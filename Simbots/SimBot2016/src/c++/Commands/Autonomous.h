/*
 * Autonomous.h
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_AUTONOMOUS_H_
#define SRC_COMMANDS_AUTONOMOUS_H_

#include <Commands/CommandGroup.h>

class Autonomous: public CommandGroup {
public:
	Autonomous();
	void Interrupted();
	void Cancel();
};

#endif /* SRC_COMMANDS_AUTONOMOUS_H_ */
