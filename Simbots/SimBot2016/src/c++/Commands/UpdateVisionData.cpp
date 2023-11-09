/*
 * UpdateVisionData.cpp
 *
 *  Created on: Jun 22, 2016
 *      Author: dean
 */

#include <Commands/UpdateVisionData.h>
#include "Robot.h"

UpdateVisionData::UpdateVisionData() {
    Requires(Robot::vision.get());
}

void UpdateVisionData::Initialize() {
    //Robot::vision.get()->Update();
}

bool UpdateVisionData::IsFinished() {
    return false;
}

void UpdateVisionData::Execute() {
    Robot::vision.get()->Update();
}

void UpdateVisionData::End() {
}
