// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TargetMgr;

import static frc.robot.Constants.*;


public class DriveBack extends CommandBase {
  // private final ProfiledPIDController m_xController =
  //     new ProfiledPIDController(20,0.0,0,
  //         new TrapezoidProfile.Constraints(
  //           kMaxVelocity, kMaxAcceleration));
  private PIDController m_xController = new PIDController(1, 0, 0);
  Drivetrain m_drive;
  double m_target;
  int count = 0;
  double error;
  /** Creates a new DriveBack. 
   * @param i
   * @param m_drive 
   * */
  public DriveBack(Drivetrain drive, double i) {
    m_drive = drive;
    m_target = i;
    addRequirements(m_drive);
  }

  public DriveBack(Drivetrain drive) {
    m_drive = drive;
    if(TargetMgr.FRCfield()){
      if(TargetMgr.getStartPosition()==TargetMgr.OUTSIDE)
        m_target=-Units.inchesToMeters(150);
      else if(TargetMgr.getStartPosition()==TargetMgr.INSIDE)
        m_target=-Units.inchesToMeters(150);
      else{
        m_xController.setPID(0.5, 0, 0.3);
        m_target=-Units.inchesToMeters(100);
      }
    }
    addRequirements(m_drive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry();
    m_xController.setTolerance(0.05, .08);
    //m_xController.setSetpoint(m_target);
    System.out.println("DriveBack.initialize target="+m_target);
    m_drive.startAuto();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_drive.getPose().getX();
    error = -0.5*kMaxVelocity;//m_xController.calculate(x, m_target);
    if((count % 20) == 0)
      System.out.format("x: %-1.2f target: %-1.2f correction: %-1.2f\n", x, m_target, error);
    count++;
    m_drive.drive(error, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double x=m_drive.getPose().getX();
    System.out.println("DriveBack.end x="+x);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double x=m_drive.getPose().getX();
    return Math.abs(x-m_target)<0.05?true:false;

    //return m_xController.atSetpoint();
  }
}
