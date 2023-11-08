#ifndef Turn_H
#define Turn_H

#include "../CommandBase.h"

class TurnForTime : public CommandBase {
	double speed;
	double time;
	double targetTime;
	double currentTime;
public:
	TurnForTime(double t, double s);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Turn_H
