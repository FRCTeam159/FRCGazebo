// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class OpenGate extends CommandBase {
  /** Creates a new OpenGate. */
  public OpenGate() {
    addRequirements(Robot.holder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.holder.openGate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.holder.openGate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.holder.isGateOpen();
  }
}
