// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ControlArm extends CommandBase implements Constants {
  Arm m_arm;
  XboxController m_controller;
  /** Creates a new ControlArm. 
   * @param m_controller */
  public ControlArm(Arm arm, XboxController controller) {
    m_arm=arm;
    m_controller=controller;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left=m_controller.getLeftTriggerAxis();
    double right=m_controller.getRightTriggerAxis();

    if(m_controller.getRightBumperPressed())
      m_arm.togglePusher();
    else if(m_controller.getLeftBumperPressed())
      m_arm.toggleShooter();
    else if(m_controller.getXButtonPressed())
      m_arm.togglePickup();
    else if(m_controller.getAButtonPressed())
      m_arm.setTargetAngle(PICKUP_ANGLE);
    else if(m_controller.getBButtonPressed())
      m_arm.setTargetAngle(AMP_SHOOT_ANGLE);
    else if(m_controller.getYButtonPressed())
      m_arm.setTargetAngle(SPEAKER_SHOOT_ANGLE);
    else if (left > 0)
        m_arm.stepDown(left);
    else if (right > 0)
        m_arm.stepUp(right);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}