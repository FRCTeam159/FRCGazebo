/*
 * InitShooter.h
 *
 *  Created on: Jun 23, 2016
 *      Author: dean
 */

#ifndef SRC_COMMANDS_INITSHOOTER_H_
#define SRC_COMMANDS_INITSHOOTER_H_

#include <Commands/Command.h>

class InitShooter: public Command {
public:
    InitShooter();
    void Initialize();
    void Execute();
    bool IsFinished();
    void End();
    void Interrupted(){End();}
};

#endif /* SRC_COMMANDS_INITSHOOTER_H_ */
