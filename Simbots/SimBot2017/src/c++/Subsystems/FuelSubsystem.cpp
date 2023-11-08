#include <Subsystems/FuelSubsystem.h>
#include "RobotMap.h"
#include "Commands/FuelMonitor.h"

#define PUSHER_SPEED 0.5
FuelSubsystem::FuelSubsystem() : Subsystem("FuelSubsystem"),
	fuelPusherMotor(FUELMOTOR) {
	fuelPusherMotor.ConfigLimitMode(CANTalon::kLimitMode_SwitchInputsOnly);
	fuelPusherMotor.ConfigRevLimitSwitchNormallyOpen(false);
	fuelPusherMotor.ConfigFwdLimitSwitchNormallyOpen(false);
}

void FuelSubsystem::InitDefaultCommand() {
	Enable();
	SetDefaultCommand(new FuelMonitor());
}

void FuelSubsystem::SetVoltage(double value) {
	fuelPusherMotor.Set(value);
}

bool FuelSubsystem::AtUpperLimit() {
	return fuelPusherMotor.IsFwdLimitSwitchClosed();
}

bool FuelSubsystem::AtLowerLimit() {
	return fuelPusherMotor.IsRevLimitSwitchClosed();
}

void FuelSubsystem::Disable() {
	fuelPusherMotor.Disable();
}

void FuelSubsystem::Enable() {
	fuelPusherMotor.Enable();
}
