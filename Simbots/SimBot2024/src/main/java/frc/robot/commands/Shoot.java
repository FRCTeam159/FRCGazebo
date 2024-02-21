// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class Shoot extends Command implements Constants{
  /** Creates a new Shoot. 
   * @param m_arm 
   * */
  Arm m_arm;
  Drivetrain m_drive;
  boolean shooter_ready=false;
  boolean shooting=false;
  Timer m_timer=new Timer();
  boolean ok2shoot=false;
  public Shoot(Drivetrain drive, Arm arm) {
    m_arm=arm;
    m_drive=drive;
    addRequirements(arm);
    m_timer.start();
    ok2shoot=Arm.noteAtIntake();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shoot.init");
    shooter_ready=false;
    shooting=false;
    ok2shoot=false;
    m_timer.reset();
    m_arm.setShooterOn();
    m_arm.setPickupOff();
    ok2shoot=Arm.noteAtIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!shooter_ready && m_arm.atTargetSpeed()){
      shooter_ready=true;
      m_arm.setPickupOn();
      m_timer.reset();
      shooting=true;
      Robot.status="Shooting";
    }
    m_drive.drive(-0.1,0,0,false); // align bumpers with platform
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     System.out.println("Shoot.end");
    m_arm.setShooterOFf();
    //m_arm.setTargetAngle(PICKUP_ANGLE);
    Robot.status="End";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shooting && m_timer.get()>3 && !Arm.noteAtIntake())
      return true;
    if(shooting && m_timer.get()>5)
      return true;
    if(!ok2shoot)
      return true;
    return false;
  }
}
