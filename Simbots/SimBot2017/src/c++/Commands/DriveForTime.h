#ifndef DriveForTime_H
#define DriveForTime_H

#include "CommandBase.h"
#include "WPILib.h"

class DriveForTime: public CommandBase
{
private:
	double speed;
	double time;
	double targetTime;
	double currentTime;
public:
	DriveForTime(double t, double s);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	void Drive(float xAxis, float yAxis, float zAxis);
};

#endif
