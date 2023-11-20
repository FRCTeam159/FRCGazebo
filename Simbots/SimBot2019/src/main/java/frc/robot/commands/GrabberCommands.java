/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Button;
import edu.wpi.first.wpilibj.XboxController;

public class GrabberCommands extends CommandBase implements RobotMap{
  Button clawButton=new Button(ARMS_TOGGLE_BUTTON);
  Button tiltButton = new Button(GRABBER_TILT_BUTTON);

  public GrabberCommands() {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(Robot.grabber);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("GrabberCommands.initialize()");
    //System.out.println("GRABBER_TILT_BUTTON="+GRABBER_TILT_BUTTON+" id="+tiltButton.getID());

    //Robot.grabber.closeClaw();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    XboxController stick =Robot.controller;
    boolean intakeButtonPressed = stick.getRawButton(INTAKE_BUTTON);
    boolean outputButtonPressed = stick.getRawButton(OUTPUT_BUTTON);
    
    if (clawButton.isPressed()){
      if (Robot.grabber.isClawOpen())
        Robot.grabber.closeClaw();
      else 
        Robot.grabber.openClaw();
    }
    if (tiltButton.isPressed()){
      if(Robot.grabber.isTilted())
        Robot.grabber.tilt(true);
      else
        Robot.grabber.tilt(false);
    }
    if (intakeButtonPressed)
      Robot.grabber.grab();
    else if (outputButtonPressed)
      Robot.grabber.eject();
    else 
      Robot.grabber.hold();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted){
    System.out.println("GrabberCommands.end()");
  }

}
