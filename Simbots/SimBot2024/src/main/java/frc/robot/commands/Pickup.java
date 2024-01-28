// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class Pickup extends CommandBase implements Constants{
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  private final Arm m_arm;
  double timeout;
  boolean resetting=false;
  double starttm;

  public Pickup(Drivetrain drive, Arm arm, double tm) {
    m_drive=drive;
    m_arm=arm;
    timeout=tm;
    m_timer.start();
    
    //m_drive.resetPose();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    m_timer.reset();
    starttm=0;//m_drive.getClockTime();
    //m_drive.resetPose();
    System.out.println("Pickup.init");
    m_arm.setPickupOn();
  
    m_drive.resetWheels();
    Arm.status="Pickup";
    resetting=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_drive.wheelsReset()){
      m_drive.allignWheels();
    }
    // m_drive.drive(0,0,0,false);
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_drive.disable();
    m_drive.resetPose();
    m_arm.setTargetAngle(SPEAKER_SHOOT_ANGLE);
    m_arm.setPickupOff();
   // m_drive.driveForward(0.1);
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
    if(m_drive.wheelsReset()){
      System.out.println("Pickup - wheels alligned at:"+(tm-starttm>timeout));
      return true;
    }
    if(m_arm.isNoteCaptured()){
      System.out.println("Pickup - note captured");
      return true;
    }
    //return m_timer.get()>=timeout;
    return false;

  }
}
