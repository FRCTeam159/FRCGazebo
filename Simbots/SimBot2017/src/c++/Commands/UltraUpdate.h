#ifndef UltraUpdate_H
#define UltraUpdate_H

#include "../CommandBase.h"

class UltraUpdate : public CommandBase {
public:
	UltraUpdate();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // UltraUpdate_H
