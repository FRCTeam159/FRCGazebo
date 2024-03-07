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
import frc.robot.subsystems.DualCameras;

public class Pickup extends Command implements Constants{
  private final Arm m_arm;
  boolean note_captured=false; 
  boolean note_detected=false;
  Timer m_timer=new Timer();

  public Pickup(Arm arm) {
    m_arm=arm;
    m_timer.start();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Autonomous.log("Pickup.init");
    m_arm.setPickupOn();
    Robot.status="Pickup";
    note_captured=false;
    note_detected=false;

    DualCameras.setNoteCamera();

    m_timer.reset();
    m_arm.setTargetAngle(Constants.PICKUP_ANGLE);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!note_detected && Arm.noteAtIntake() && m_arm.atTargetAngle()){
      Autonomous.log("Pickup - note detected");
      m_timer.reset();
      note_detected=true;
    }   
    if(!note_captured && note_detected && m_timer.get()>0.5){
      Autonomous.log("Pickup - note captured");
      m_arm.setTargetAngle(SPEAKER_SHOOT_ANGLE); // lift note off the ground
      note_captured=true; 
      m_timer.reset();  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Autonomous.log("Pickup.end");
    m_arm.setPickupOff();   
    m_arm.setTargetAngle(SPEAKER_SHOOT_ANGLE); // lift note off the ground
    DualCameras.setTagCamera();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if (!Autonomous.okToRun())
      return true; 
    if(note_captured && Arm.noteAtShooter())
      return true;
    return false;
  }
}
