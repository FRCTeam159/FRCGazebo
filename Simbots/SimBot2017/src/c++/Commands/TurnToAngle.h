#ifndef TurnToAngle_H
#define TurnToAngle_H

#include "../CommandBase.h"

class TurnToAngle : public CommandBase, public PIDSource, public PIDOutput {
	PIDController pid;
	double angle=0;
	double radius=1;
public:
	TurnToAngle(double a);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
    double PIDGet();
    void PIDWrite(double d);
};

#endif  // TurnToAngle_H
