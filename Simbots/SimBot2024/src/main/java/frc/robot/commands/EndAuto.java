// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class EndAuto extends Command implements Constants{
  /** Creates a new InitArm. */
  Drivetrain m_drive;
  public EndAuto(Drivetrain drive) {
    m_drive=drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.endAuto();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Autonomous.end(); 
    Robot.status="Auto End";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
