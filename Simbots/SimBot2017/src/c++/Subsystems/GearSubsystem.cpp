#include "GearSubsystem.h"
#include "RobotMap.h"

GearSubsystem::GearSubsystem() : Subsystem("GearSubsystem"),
	piston(1,1){
	frc::SmartDashboard::PutBoolean("GearPlateOpen", isOpen);
}

void GearSubsystem::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void GearSubsystem::Open() {
	piston.Set(true);
	isOpen=true;
	frc::SmartDashboard::PutBoolean("GearPlateOpen", isOpen);
}

void GearSubsystem::Close() {
	piston.Set(false);
	isOpen=false;
	frc::SmartDashboard::PutBoolean("GearPlateOpen", isOpen);

}
// Put methods for controlling this subsystem
// here. Call these from Commands.
