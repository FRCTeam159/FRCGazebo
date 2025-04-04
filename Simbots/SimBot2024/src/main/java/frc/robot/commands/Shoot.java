// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class Shoot extends Command implements Constants{
  /** Creates a new Shoot. 
   * @param m_arm 
   * */
  Drivetrain m_drive;
  Arm m_arm;
  boolean m_shooter_ready=false;
  boolean m_shooting=false;
  Timer m_timer=new Timer();
  boolean m_note_at_start=false;
  public Shoot(Arm arm, Drivetrain drive) {
    m_arm=arm;
    m_drive=drive;
    addRequirements(arm);
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Autonomous.log("Shoot.init");
    m_shooter_ready=false;
    m_shooting=false;
    m_note_at_start=Arm.noteAtIntake();
    m_timer.reset();
    m_arm.setShooterOn();
    m_arm.setPickupOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(!m_shooter_ready && m_arm.atTargetSpeed()){
      m_shooter_ready=true;
      m_arm.setPushOn();
      m_timer.reset();
      m_shooting=true;
      Robot.status="Shooting";
    }
    m_drive.drive(0.001,0,0,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Autonomous.log("Shoot.end");
    m_arm.setShooterOFf();
    m_arm.setPushOFf();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Autonomous.okToRun())
      return true;
    if(m_shooter_ready && m_timer.get()>1 && !Arm.noteAtShooter()){
      Autonomous.log("Shoot - shot delivered");
      return true;
    }
    if(m_shooting && m_timer.get()>5) {// taking too long - something isn't right
      Autonomous.log("Shoot - timed out");
      return true;
     }
    if(!m_note_at_start){
      Autonomous.log("Shoot - no note at start - aborting");
      return true;
    }
    return false;
  }
}
