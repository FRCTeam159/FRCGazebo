/*
 * InitShooter.cpp
 *
 *  Created on: Jun 23, 2016
 *      Author: dean
 */

#include <Commands/InitShooter.h>
#include "Robot.h"

InitShooter::InitShooter() {
    Requires(Robot::shooter.get());
    std::cout << "new InitShooter()"<< std::endl;
}

void InitShooter::Initialize() {
    std::cout << "InitShooter started .."<<std::endl;
    if(!Robot::shooter->IsInitialized())
        Robot::shooter->Initialize();
}

void InitShooter::Execute(){
    Robot::shooter->Execute();
}

bool InitShooter::IsFinished() {
    return Robot::shooter->IsInitialized();
}

void InitShooter::End() {
    std::cout << "InitShooter End"<<std::endl;
}

