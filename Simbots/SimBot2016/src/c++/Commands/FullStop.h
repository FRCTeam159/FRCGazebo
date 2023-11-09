/*
 * FullStop.h
 *
 *  Created on: Mar 9, 2016
 *      Author: dean
 */

#ifndef SRC_COMMANDS_FULLSTOP_H_
#define SRC_COMMANDS_FULLSTOP_H_

#include <Commands/Command.h>

class FullStop: public Command {
public:
	FullStop();
	void Initialize();
	bool IsFinished();
	void Execute() {}
	void End();
	void Interrupted() { End();}

};

#endif /* SRC_COMMANDS_FULLSTOP_H_ */
