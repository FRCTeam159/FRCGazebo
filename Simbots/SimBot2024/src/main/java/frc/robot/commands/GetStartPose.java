// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TargetMgr;

public class GetStartPose extends Command implements Constants{
  /** Creates a new InitArm. */

  Arm m_arm;
  public GetStartPose(Arm arm) {
    m_arm=arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.status="Auto Start";
    TargetMgr.clearStartPose();
    m_arm.setTargetAngle(Constants.SPEAKER_SHOOT_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atTargetAngle()&&TargetMgr.startPoseSet();
  }
}
