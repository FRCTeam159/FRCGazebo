#ifndef GearSubsystem_H
#define GearSubsystem_H

#include <Commands/Subsystem.h>
#include "WPILib.h"

class GearSubsystem : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	Solenoid piston;
	bool isOpen=false;
public:
	GearSubsystem();
	void InitDefaultCommand();

	void Open();
	void Close();
	bool IsOpen() { return isOpen;}
};

#endif  // GearSubsystem_H
