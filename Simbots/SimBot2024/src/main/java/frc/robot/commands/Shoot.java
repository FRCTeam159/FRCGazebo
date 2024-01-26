// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class Shoot extends CommandBase implements Constants{
  /** Creates a new Shoot. 
   * @param m_arm */
  Arm m_arm;
  boolean shooter_ready=false;
  boolean shooting=false;
  Timer m_timer=new Timer();
  public Shoot(Arm arm) {
    m_arm=arm;
    addRequirements(arm);
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_arm.setShooterOn();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!shooter_ready && m_arm.atTargetSpeed()){
      shooter_ready=true;
      m_arm.setPusherOn();
      m_timer.reset();
      shooting=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setShooterOFf();
    m_arm.setPusherOFf();
    m_arm.setTargetAngle(PICKUP_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shooting && m_timer.get()>5)
      return true;
    return false;
  }
}
