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

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(0.5);


  public static final int LEFT_JOYSTICK = 1;
  public static final int RIGHT_JOYSTICK = 4;

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
    //m_drive.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double zs = -m_controller.getRawAxis(LEFT_JOYSTICK);
    double xs = m_controller.getRawAxis(RIGHT_JOYSTICK);


      // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.05))
            * Drivetrain.kMaxVelocity;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.05))
            * Drivetrain.kMaxVelocity;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.05))
            * Drivetrain.kMaxAngularSpeed;
   
    // if (Math.abs(zs) < 0.2)
    //   zs = 0;
    // if (Math.abs(xs) < 0.2) 
    //   xs = 0;
    if(!started && (Math.abs(zs)>0 || Math.abs(xs)>0)){
      m_drive.enable();
      started=true;
    }
    //m_drive.testDrive(zs, xs);
    //m_drive.testDrive(xSpeed, rot);
    //  m_drive.turnInPlace(xSpeed);

    //m_drive.drive(, ySpeed,rot,false);
    m_drive.drive(xSpeed, 0,rot,false);

   // System.out.println(xSpeed+" "+ySpeed+" "+rot+" "+zs+" "+xs);

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
