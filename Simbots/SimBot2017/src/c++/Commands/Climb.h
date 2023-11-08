#ifndef Climb_H
#define Climb_H

#include "../CommandBase.h"

class Climb : public CommandBase {
	enum {
			WAITFORTOP,
			REACHEDTOP,
		};
	int state = WAITFORTOP;
public:
	Climb();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Climb_H
