// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class Pickup extends Command implements Constants{
  private final Timer m_timer = new Timer();
  private final Arm m_arm;
  double timeout;
  boolean resetting=false;
  double starttm;

  public Pickup(Arm arm, double tm) {
    m_arm=arm;
    timeout=tm;
    m_timer.start();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    starttm=0;
    System.out.println("Pickup.init");
    m_arm.setPickupOn();

    Arm.status="Pickup";
    resetting=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.get()>0.5*timeout)
      m_arm.setTargetAngle(SPEAKER_SHOOT_ANGLE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setTargetAngle(SPEAKER_SHOOT_ANGLE);
    m_arm.setPickupOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double tm=m_timer.get();
    if(tm-starttm>timeout){
      System.out.println("Pickup - timout expired");
      Autonomous.ok2run=false;
      return true;
    } 
    if(m_arm.isNoteCaptured()){
      System.out.println("Pickup - note captured");
      return true;
    }
    return false;
  }
}
