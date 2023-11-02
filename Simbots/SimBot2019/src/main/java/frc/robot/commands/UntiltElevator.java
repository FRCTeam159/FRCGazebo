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

public class UntiltElevator extends CommandBase {
    Timer timer=new Timer();
    double timeout;
  
  public UntiltElevator(double delay) {
    timeout=delay;
    timer.start();
    addRequirements(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    timer.reset();
    //Robot.elevator.enable();
    System.out.println("UntiltElevator initialized");
    //goToHatchHeight();
    Robot.elevator.tiltElevator(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // try {
    //   Thread.sleep(2000);
    // } catch (InterruptedException ex) {
    //   System.out.println("exception)");
    // }
    //Robot.elevator.tiltElevator(true);
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
    System.out.println("UntiltElevator end");
  }

}
