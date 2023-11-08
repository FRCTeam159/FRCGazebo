#include <memory>

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "CommandBase.h"
#include "Commands/Autonomous.h"

#include <thread>
#include <CameraServer.h>
#include <Commands/TurnForTime.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include "Commands/DriveToTarget.h"
#include "Commands/DriveForTime.h"
#include "Commands/DriveStraight.h"
#include "Commands/TurnToAngle.h"
#include "Commands/TurnForTime.h"
#include "Commands/DeliverGear.h"
#include "Commands/DrivePath.h"


#define TUNE_AUTO
#define DRIVE_TIME 3.0
#define TURN_TIME 1.0
#define TURNANGLE 60
#define DRIVE_FORWARD 0.2
#define DRIVE_BACKWARD -0.1

#define DRIVEDISTANCE 5.5*12  // distance to baseline in inches (from robot center)

static double rightDrive=0.47;
static double rightTurn=0.4;
static double leftDrive=0.6;
static double leftTurn=0.5;

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() override {
		CommandBase::RobotInit();
		frc::SmartDashboard::PutString("AutoMode", "PFRight");
		frc::SmartDashboard::PutBoolean("Targeting", false);

		frc::SmartDashboard::PutNumber("leftDrive", leftDrive);
		frc::SmartDashboard::PutNumber("leftTurn",leftTurn);
		frc::SmartDashboard::PutNumber("rightDrive", rightDrive);
		frc::SmartDashboard::PutNumber("rightTurn",rightTurn);
	}
	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	void DisabledInit() override {
		CommandBase::DisabledInit();
	}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}


	void AutonomousInit() override {
		std::string autoSelected = frc::SmartDashboard::GetString("AutoMode", "PFRight");

		rightDrive = frc::SmartDashboard::GetNumber("rightDrive", rightDrive);
		rightTurn = frc::SmartDashboard::GetNumber("rightTurn",rightTurn);
		leftDrive = frc::SmartDashboard::GetNumber("leftDrive",leftDrive);
		leftTurn = frc::SmartDashboard::GetNumber("leftTurn",leftTurn);
		CommandGroup *autonomous=new Autonomous();
		if (autoSelected == "Right") {
			// practice-bot: leftDrive=0.45 turnVoltage=0.32
			autonomous->AddSequential(new DriveForTime(DRIVE_TIME,rightDrive));
			autonomous->AddSequential(new TurnForTime(TURN_TIME, -rightTurn));
	        autonomous->AddSequential(new DriveForTime(0.5, 0));  // pause to let image frames catch up
			autonomous->AddSequential(new DeliverGear());
			cout<<"Right Auto"<<endl;
		}
		else if(autoSelected == "Left"){
			// practice-bot: leftDrive=0.45 turnVoltage=0.25
			autonomous->AddSequential(new DriveForTime(DRIVE_TIME,leftDrive));
			autonomous->AddSequential(new TurnForTime(TURN_TIME, leftTurn));
            autonomous->AddSequential(new DriveForTime(0.5, 0));  // pause to let image frames catch up
			autonomous->AddSequential(new DeliverGear());
			cout<<"Left Auto"<<endl;
		}
		else if(autoSelected == "Center") {
			cout<<"Center Auto"<<endl;
			autonomous->AddSequential(new DriveForTime(1.5, 0.4));
			autonomous->AddSequential(new DeliverGear());
		}
		else if(autoSelected == "PFLeft") {
			cout<<"Pathfinder Test left"<<endl;
			autonomous->AddSequential(new DrivePath(DrivePath::LEFT));
			//autonomous->AddSequential(new DeliverGear());
		}
		else if(autoSelected == "PFRight") {
			cout<<"Pathfinder Test right"<<endl;
			autonomous->AddSequential(new DrivePath(DrivePath::RIGHT));
			//autonomous->AddSequential(new DeliverGear());
		}
		else if(autoSelected == "PFCenter") {
			cout<<"Pathfinder Test center"<<endl;
			autonomous->AddSequential(new DrivePath(DrivePath::CENTER));
			//autonomous->AddSequential(new DeliverGear());
		}
		else{
			cout<<"Auto Mode Disabled"<<endl;
		}
		autonomousCommand.reset(autonomous);

		CommandBase::AutonomousInit();

		if (autonomousCommand.get() != nullptr) {
			autonomousCommand->Start();
		}
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != nullptr) {
			autonomousCommand->Cancel();
		}
		CommandBase::TeleopInit();

	}

	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TestInit(){
	}

	void TestPeriodic() override {
		frc::LiveWindow::GetInstance()->Run();
	}

private:
	std::unique_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<Command*> chooser;
	std::unique_ptr<frc::Command> disabledCommand;
};

START_ROBOT_CLASS(Robot)
