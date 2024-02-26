// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class StartAuto extends Command {
  Drivetrain m_drive;
  Timer m_timer=new Timer();
  /** Creates a new StartAuto. */
  public StartAuto(Drivetrain drive) {
    m_drive=drive;
    m_timer.start();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_drive.startAuto();
    m_drive.resetWheels(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.alignWheels();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drive.resetPose();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.wheelsReset() || m_timer.get()>2;
  }
}
