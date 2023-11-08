#ifndef Fuel_H
#define Fuel_H

#include <Commands/Subsystem.h>
#include "CANTalon.h"

class FuelSubsystem : public Subsystem {
private:
	CANTalon fuelPusherMotor;

public:
	FuelSubsystem();
	void InitDefaultCommand();
	void SetVoltage(double);
	bool AtUpperLimit();
	bool AtLowerLimit();

	void Enable();
	void Disable();

};

#endif  // Fuel_H
