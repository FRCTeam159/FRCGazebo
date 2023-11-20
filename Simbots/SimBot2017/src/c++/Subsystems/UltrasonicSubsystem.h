#ifndef UltrasonicSubsystem_H
#define UltrasonicSubsystem_H

#include <Commands/Subsystem.h>
#include "WPILib.h"

#ifndef SIMULATION
#include "SerialPort.h"
#include "DigitalOutput.h"
#include "I2C.h"
#endif

class UltrasonicSubsystem : public Subsystem {
private:
	bool portEnabled;
	double distance = 0;
#ifdef SIMULATION
	AnalogInput rangefinder;
#else
	SerialPort port;
	DigitalOutput dOutput;
#endif

public:
	UltrasonicSubsystem();
	void InitDefaultCommand();
	void Disable();
	void Enable();
	bool IsEnabled();
	double GetDistance();
	void Init();
};

#endif  // UltrasonicSubsystem_H
