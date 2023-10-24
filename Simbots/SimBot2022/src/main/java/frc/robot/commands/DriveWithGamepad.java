// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveWithGamepad extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drive;
  private final XboxController m_controller;
  private boolean started=false;

  public static final int LEFT_JOYSTICK = 1;
  public static final int RIGHT_JOYSTICK = 4;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param m_Controller
   */
  public DriveWithGamepad(DriveTrain subsystem, XboxController controller) {
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
    double zs = m_controller.getRawAxis(LEFT_JOYSTICK);
    double xs = m_controller.getRawAxis(RIGHT_JOYSTICK);
 

    double zoff=0.35;

    if (Math.abs(zs) < zoff) 
      zs = 0;
    else if(zs>0)
      zs=(zs-zoff)/(1-zoff);

    if (Math.abs(xs) < 0.1) 
      xs = 0;
    if(!started && (Math.abs(zs)>0 || Math.abs(xs)>0)){
      m_drive.enable();
      started=true;
    }

    if(m_drive.arcade_mode)
      m_drive.arcadeDrive(-zs, -xs);
    else
      m_drive.odometryDrive(-zs,xs);
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
