/*
 * Robot.h
 *
 *  Created on: Jun 3, 2014
 *      Author: alex
 */

#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

#include "WPILib.h"
#include "Commands/Command.h"
#include "Commands/Autonomous.h"

#include <Subsystems/DriveTrain.h>
#include <Subsystems/Holder.h>
#include <Subsystems/Shooter.h>
#include <Subsystems/Loader.h>
#include <Subsystems/Vision.h>
#include <Subsystems/VisionProcess.h>

#include "OI.h"

class Robot: public IterativeRobot {
	private:
	LiveWindow *lw = LiveWindow::GetInstance();
	void RobotInit();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void TestPeriodic();
	void DisabledInit();
	void DisabledPeriodic();
public:
	static std::shared_ptr<DriveTrain> drivetrain;
	static std::shared_ptr<Holder> holder;
	static std::shared_ptr<Shooter> shooter;
	static std::shared_ptr<Loader> loader;
	static std::shared_ptr<OI> oi;
    static std::shared_ptr<Vision> vision;
    static std::shared_ptr<VisionProcess> visionprocess;

	static std::unique_ptr<Autonomous> autonomous;
	static void SetMode(int m);
	static int GetMode();
};


#endif /* ROBOT_H_ */
