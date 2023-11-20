#include <CameraServer.h>
#include <Commands/VisionUpdate.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include "Subsystems/GripPipeline.h"

VisionUpdate::VisionUpdate() {
	Requires(visionSubsystem.get());
	std::cout << "new VisionUpdate"<< std::endl;
}

// Called just before this Command runs the first time
void VisionUpdate::Initialize() {
	cout << "VisionUpdate Started .."<<endl;
}

// Called repeatedly when this Command is scheduled to run
void VisionUpdate::Execute() {
	visionSubsystem->Process();
}

// Make this return true when this Command no longer needs to run execute()
bool VisionUpdate::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void VisionUpdate::End() {
	cout << "VisionUpdate End"<<endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void VisionUpdate::Interrupted() {
	End();
}
