#ifndef DriveWithJoystick_H
#define DriveWithJoystick_H

#include "../CommandBase.h"
#include "WPILib.h"

class DriveWithJoystick: public CommandBase
{
public:
	DriveWithJoystick();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	float quadDeadband(float minThreshold, float minOutput, float input);
};

#endif
