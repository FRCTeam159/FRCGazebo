#ifndef ClimbingSubsystem_H
#define ClimbingSubsystem_H
#include "WPILib.h"
#include "Commands/Subsystem.h"
#include "CANTalon.h"
#include <Commands/Subsystem.h>

class ClimbingSubsystem : public Subsystem {

private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	CANTalon climberMotor;

public:
	ClimbingSubsystem();
	void InitDefaultCommand();
	bool IsAtTop();
	double GetCurrent();
	void Stop();
	void ClimberClimb();
};

#endif  // ClimbingSubsystem_H
