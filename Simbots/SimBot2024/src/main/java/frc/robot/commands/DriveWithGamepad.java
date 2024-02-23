// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagDetector;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveWithGamepad extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drive;
  private final XboxController m_controller;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(0.2);
  boolean m_aligning = false;
  AlignWheels m_align = null;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem    The subsystem used by this command.
   * @param m_Controller
   */
  public DriveWithGamepad(Drivetrain subsystem, XboxController controller) {
    m_drive = subsystem;
    m_controller = controller;
    m_align = new AlignWheels(m_drive, 2.0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double now = 0;// WPIUtilJNI.now() * 1e-6;
    m_xspeedLimiter.reset(now);
    m_yspeedLimiter.reset(now);
    m_rotLimiter.reset(now);

    System.out.println("DriveWithGampad started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double vx = m_controller.getLeftY();
    double vy = m_controller.getLeftX();
    double vr = m_controller.getRightX();
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(vx, 0.3))
        * Drivetrain.kMaxVelocity;

    // Get the y speed or sideways/strafe speed.
    final var ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(vy, 0.3))
        * Drivetrain.kMaxVelocity;

    // Get the rate of angular rotation.
    final var rot = m_rotLimiter.calculate(MathUtil.applyDeadband(vr, 0.3))
        * Drivetrain.kMaxAngularVelocity;

    if (m_drive.disabled())
      m_drive.enable();
    if (m_controller.getRightStickButtonPressed()) {
      System.out.println("Aligning");
      if (!m_aligning) {
        m_align.initialize();
        m_aligning = true;
      } else
        m_aligning = false;
    }
    if (m_aligning)
      align();
    boolean do_targeting = TagDetector.isTargeting();
    if (!m_aligning && !do_targeting)
      m_drive.drive(xSpeed, -ySpeed, -rot, m_drive.fieldOriented());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveWithGampad cancelled");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  void align() {
    if (m_align.isFinished()) {
      m_align.end(false);
      m_aligning = false;
    } else
      m_align.execute();
  }

}
