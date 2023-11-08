#ifndef DriveToTarget_H
#define DriveToTarget_H

#include "../CommandBase.h"

class DriveToTarget : public CommandBase, public PIDSource, public PIDOutput{
	PIDController pid;
	double distance;
	double angle;
	//Vision::TargetInfo target;
	double GetDistance();
	bool error=false;

public:
	DriveToTarget();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
    double PIDGet();
    void PIDWrite(double d);
};

#endif  // DriveToTarget_H
