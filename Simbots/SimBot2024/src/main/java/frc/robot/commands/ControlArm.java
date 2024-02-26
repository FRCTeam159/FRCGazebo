// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class ControlArm extends Command implements Constants {
  Arm m_arm;
  Drivetrain m_drive;
  XboxController m_controller;
  Shoot shoot;
  Pickup pickup;
  AutoTarget target;
  boolean m_shooting = false;

  boolean m_grabbing = false;
  boolean m_targeting = false;

  /**
   * Creates a new ControlArm.
   * 
   * @param m_controller
   */
  public ControlArm(Arm arm, Drivetrain drive, XboxController controller) {
    m_arm = arm;
    m_drive = drive;
    m_controller = controller;
    shoot = new Shoot(arm, drive);
    pickup = new Pickup(arm);
    target = new AutoTarget(m_arm, m_drive);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooting = false;
    m_grabbing = false;
    m_targeting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = m_controller.getLeftTriggerAxis();
    double right = m_controller.getRightTriggerAxis();

    if (m_controller.getRightBumperPressed()) {
      if (!m_shooting) {
        shoot.initialize();
        m_shooting = true;
      } else
        m_shooting = false;
    } else if (m_controller.getLeftBumperPressed()) {
      if (!m_grabbing) {
        pickup.initialize();
        m_grabbing = true;
      } else
        m_grabbing = false;
    } else if (m_controller.getXButtonPressed()) {
      if (!m_targeting) {
        target.initialize();
        m_targeting = true;
      } else
        m_targeting = true;
    } else if (m_controller.getAButtonPressed())
      m_arm.setTargetAngle(PICKUP_ANGLE);
    else if (m_controller.getBButtonPressed())
      m_arm.setTargetAngle(AMP_SHOOT_ANGLE);
    else if (m_controller.getYButtonPressed())
      m_arm.setTargetAngle(SPEAKER_SHOOT_ANGLE);
    else if (left > 0)
      m_arm.stepDown(left);
    else if (right > 0)
      m_arm.stepUp(right);

    if (m_shooting)
      shoot();
    else if (m_grabbing)
      pickup();
    else if (m_targeting)
      target();
  }

  void shoot() {
    if (shoot.isFinished()) {
      shoot.end(false);
      m_shooting = false;
    } else
      shoot.execute();
  }

  void pickup() {
    if (pickup.isFinished()) {
      pickup.end(false);
      m_grabbing = false;
    } else
      pickup.execute();
  }

  void target() {
    if (target.isFinished()) {
      target.end(m_controller.getXButtonReleased());
      m_targeting = false;
    } else
      target.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
