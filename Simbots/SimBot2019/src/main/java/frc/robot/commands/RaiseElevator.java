/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RaiseElevator extends CommandBase {
  Timer timer=new Timer();
  double timeout;
  public RaiseElevator(double delay) {
    addRequirements(Robot.elevator);
    timer.start();
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("RaiseElevator initialized");
    Robot.elevator.resetLevel();
    timer.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Robot.elevator.log();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(timer.get()>timeout)
      return true;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted){
    System.out.println("RaiseElevator end");
  }


}
