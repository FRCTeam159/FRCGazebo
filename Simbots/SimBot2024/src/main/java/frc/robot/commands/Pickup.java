// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Pickup extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  double timeout;
  boolean resetting=false;
  double starttm;

  public Pickup(Drivetrain drive, double tm) {
    m_drive=drive;
    timeout=tm;
    m_timer.start();
    
    //m_drive.resetPose();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.reset();
    m_drive.resetPose();
    m_timer.reset();
    starttm=0;//m_drive.getClockTime();
    //m_drive.resetPose();
    System.out.println("Pickup.init");
    // SmartDashboard.putBoolean("Gazebo", false);
    // SmartDashboard.putBoolean("Reset", true);
    resetting=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_drive.updateOdometry();
   // if(m_timer.get()>1 && !resetting){
      //m_drive.resetPose();
   //   System.out.println("Pickup.started");
   //   resetting=true;
   // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.putBoolean("Gazebo", true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double tm=m_timer.get();//m_drive.getClockTime();
    if(tm-starttm>timeout){
      System.out.println("Pickup - timout expired");
      return true;
    }
    //return m_timer.get()>=timeout;
    return false;

  }
}
