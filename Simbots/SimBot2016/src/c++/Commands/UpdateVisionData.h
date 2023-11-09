/*
 * UpdateVisionData.h
 *
 *  Created on: Jun 22, 2016
 *      Author: dean
 */

#ifndef SRC_COMMANDS_UPDATEVISIONDATA_H_
#define SRC_COMMANDS_UPDATEVISIONDATA_H_

#include <Commands/Command.h>

class UpdateVisionData: public Command {
public:
    UpdateVisionData();
    void Initialize();
    bool IsFinished();
    void Execute();
    void End();
    void Interrupted() { End();}
};

#endif /* SRC_COMMANDS_UPDATEVISIONDATA_H_ */
