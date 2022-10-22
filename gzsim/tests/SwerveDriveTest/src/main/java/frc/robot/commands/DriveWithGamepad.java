// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveWithGamepad extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drive;
  private final XboxController m_controller;
  private boolean started=false;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param m_Controller
   */
  public DriveWithGamepad(Drivetrain subsystem, XboxController controller) {
    m_drive = subsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveWithGampad started");
    started=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.1))
            * Drivetrain.kMaxVelocity;

    // Get the y speed or sideways/strafe speed. 
    final var ySpeed =
        m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.1))
            * Drivetrain.kMaxVelocity;

    // Get the rate of angular rotation. 
    final var rot =
        m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.1))
            * Drivetrain.kMaxAngularSpeed;
   
    if(!started && (Math.abs(xSpeed)>0 || Math.abs(ySpeed)>0) ||  Math.abs(rot)>0){
      m_drive.enable();
      started=true;
    }
    //m_drive.testDrive(xSpeed, rot);
    //m_drive.turnInPlace(xSpeed);

    m_drive.drive(xSpeed, ySpeed,rot,false);

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
