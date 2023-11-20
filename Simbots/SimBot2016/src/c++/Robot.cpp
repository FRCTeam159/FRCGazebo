
#include "Robot.h"
#include "Assignments.h"

#include <thread>

#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>

#define TARGET_WIDTH 20    // tape pattern width  (inches)
#define TARGET_HEIGHT 12   // tape pattern height (inches)
#define CAMERA_HOFFSET 5   // camera horizontal offset (inches)
#define CAMERA_VOFFSET 15  // target vertical offset (inches)
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define FOV 60.0

std::shared_ptr<DriveTrain> Robot::drivetrain;
std::shared_ptr<Holder> Robot::holder;
std::shared_ptr<Shooter> Robot::shooter;
std::shared_ptr<Loader> Robot::loader;
std::shared_ptr<OI> Robot::oi;
std::shared_ptr<Vision> Robot::vision;
std::shared_ptr<VisionProcess> Robot::visionprocess;


std::unique_ptr<Autonomous> Robot::autonomous;


void Robot::RobotInit() {
	std::cout << "Robot::RobotInit" << std::endl;
	holder.reset(new Holder());
	loader.reset(new Loader());
	shooter.reset(new Shooter());
	drivetrain.reset(new DriveTrain());
	oi.reset(new OI());
    vision.reset(new Vision());
    visionprocess.reset(new VisionProcess());

    vision->SetTargetSpecs(TARGET_WIDTH,TARGET_HEIGHT);
    vision->SetCameraOffsets(CAMERA_HOFFSET,CAMERA_VOFFSET);
    vision->SetCameraSpecs(IMAGE_WIDTH,IMAGE_HEIGHT,FOV);

    visionprocess->Init();

	autonomous.reset(new Autonomous());

	OI::SetMode(SHOOTING);
}

void Robot::AutonomousInit() {
	std::cout << "Robot::AutonomousInit" << std::endl;
	drivetrain->AutonomousInit();
	shooter->AutonomousInit();
	holder->AutonomousInit();
	loader->AutonomousInit();
    vision->AutonomousInit();

	autonomous->Start();
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::DisabledInit(){
	std::cout << "Robot::DisabledInit" << std::endl;
	autonomous->Cancel();
	drivetrain->DisabledInit();
	shooter->DisabledInit();
	holder->DisabledInit();
	loader->DisabledInit();
    vision->DisabledInit();
}
void Robot::DisabledPeriodic(){
}

void Robot::TeleopInit() {
	std::cout << "Robot::TeleopInit" << std::endl;
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// this line or comment it out.
	autonomous->Cancel();
	shooter->TeleopInit();
	holder->TeleopInit();
	drivetrain->TeleopInit();
	loader->TeleopInit();
    vision->TeleopInit();

}

void Robot::TeleopPeriodic() {
	// Scheduler runs "default" commands for all subsystems
	// e.g. void DriveTrain::InitDefaultCommand() {
	//	    SetDefaultCommand(new TankDriveWithJoystick()); }
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
	lw->Run();
}

void Robot::SetMode(int m) {
	OI::SetMode(m);
	if(m==LOADING){
		std::cout << "Changing Mode to Loading"<<std::endl;
		if(!holder->IsBallPresent())
			loader->LoadBall();
		drivetrain->SetReverseDriving(true);
	}
	else{
		std::cout << "Changing Mode to Shooting"<<std::endl;
		loader->SetLow();
		drivetrain->SetReverseDriving(false);
	}
}

int Robot::GetMode() {
	return OI::GetMode();
}

START_ROBOT_CLASS(Robot);

