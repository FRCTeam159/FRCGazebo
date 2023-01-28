// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithGamepad extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final Drivetrain m_drive;
  private final XboxController m_controller;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(0.5);

  private TurnToAngle m_turnaround;

  /**
   * Creates a new DriveWithGamepad command.
   *
   * @param drive      The drive subsystem used by this command.
   * @param controller XboxController
   */
  public DriveWithGamepad(Drivetrain drive, XboxController controller) {
    m_drive = drive;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double now = 0;// WPIUtilJNI.now() * 1e-6;
    m_xspeedLimiter.reset(now);
    m_yspeedLimiter.reset(now);
    m_rotLimiter.reset(now);

    System.out.println("DriveWithGampad started");

    m_drive.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vx = m_controller.getLeftY();
    double vy = m_controller.getLeftX();
    double vr = m_controller.getRightX();
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(vx, 0.2))
        * Drivetrain.kMaxVelocity;

    // Get the y speed or sideways/strafe speed.
    final var ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(vy, 0.2))
        * Drivetrain.kMaxVelocity;

    // Get the rate of angular rotation.
    var rot = m_rotLimiter.calculate(MathUtil.applyDeadband(vr, 0.2))
        * Drivetrain.kMaxAngularSpeed;
    

    // execute a 180 degree heading reversal if left (CCW) or right (CW) bumper button is pressed
    boolean rb = m_controller.getRightBumperPressed();
    boolean lb = m_controller.getLeftBumperPressed();
    if (rb || lb) {
      m_turnaround = new TurnToAngle(m_drive, rb ? 180.0 : -180.0);
      m_turnaround.initialize();
    }

    if (m_turnaround != null && !m_turnaround.isFinished())
      rot += m_turnaround.calculate(); // enable
    m_drive.drive(xSpeed, ySpeed, rot, m_drive.isFieldOriented());
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
}
