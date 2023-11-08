#include "CommandBase.h"
#include "Subsystems/DriveTrain.h"
#include <Commands/Scheduler.h>


// Initialize a single static instance of all of your subsystems. The following
// line should be repeated for each subsystem in the project.
shared_ptr<DriveTrain> CommandBase::driveTrain;
shared_ptr<GearSubsystem> CommandBase::gearSubsystem;
shared_ptr<Vision> CommandBase::visionSubsystem;
shared_ptr<UltrasonicSubsystem>CommandBase::ultrasonicSubsystem;
shared_ptr<ClimbingSubsystem> CommandBase::climbingSubsystem;
shared_ptr<FuelSubsystem>CommandBase::fuelSubsystem;

unique_ptr<OI> CommandBase::oi = std::make_unique<OI>();

CommandBase::CommandBase(const std::string &name) :
		frc::Command(name) {
}

void CommandBase::RobotInit(){
	// Create a single static instance of all of your subsystems. The following
	// line should be repeated for each subsystem in the project.
	visionSubsystem.reset(new Vision());
	driveTrain.reset(new DriveTrain());
	gearSubsystem.reset(new GearSubsystem());
	ultrasonicSubsystem.reset(new UltrasonicSubsystem());
	fuelSubsystem.reset(new FuelSubsystem());
	climbingSubsystem.reset(new ClimbingSubsystem());
	ultrasonicSubsystem->Init();
	visionSubsystem->Init();

	oi.reset(new OI());
}

void CommandBase::AutonomousInit() {
    driveTrain->InitTravel();
	driveTrain->Enable();
	ultrasonicSubsystem->Enable();
}

void CommandBase::TeleopInit() {
    driveTrain->InitTravel();
	driveTrain->Enable();
	ultrasonicSubsystem->Enable();
	fuelSubsystem->Enable();
}

void CommandBase::DisabledInit() {
	driveTrain->Disable();
	ultrasonicSubsystem->Disable();
	fuelSubsystem->Disable();

	gearSubsystem->Close();

}
