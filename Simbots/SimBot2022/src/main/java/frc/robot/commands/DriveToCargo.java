// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Shooting;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Timing;

public class DriveToCargo extends CommandBase {
  boolean test=true;
  private final Targeting m_aim;
  private final Shooting m_shoot;
  boolean have_ball;
  boolean wait_for_hold=false;
  double runtime;
  boolean haveTarget=false;
  private String last_msg;
  private boolean started=false;

  public DriveToCargo(Targeting targeting, Shooting shoot) {
    m_aim=targeting;
    m_shoot=shoot;
    addRequirements(shoot,targeting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    showStatus("DriveToCargo.init");
    have_ball=false;
    m_aim.setFrontTarget(false);
    m_shoot.setIntakeOn();
    Timing.reset();
    haveTarget=false;
    wait_for_hold=false;
    m_aim.reset();
  }

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {

    have_ball=m_shoot.isBallCaptured();
   
    if(!haveTarget && !m_aim.isFrontCamera()){ 
      if(!started)
        showStatus("DriveToCargo started " + Timing.get());
      started=true;
      m_aim.adjust();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runtime = Timing.get();
    //if (m_shoot.isBallCaptured())
     // m_shoot.setIntakeHold();
    //else
    //  m_shoot.setIntakeOff();
    m_aim.disable();
    Autonomous.totalRuntime += runtime;
    if(Autonomous.autoFailed)
      showStatus("DriveToCargo.end - Auto failed");
    else
      showStatus("DriveToCargo.end " + runtime + " total time: " + Autonomous.totalRuntime);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Autonomous.autoFailed)
      return true;
    if(!m_aim.backCamera( )|| !started)
      return false;
    if(have_ball && !wait_for_hold){
      m_aim.disable();
      m_shoot.setIntakeHold();
      wait_for_hold=true;
    }
    if(wait_for_hold && m_shoot.intakeReady(Shooting.kIntakeHold))
      return true;

    boolean onTarget = m_aim.onTarget();
    if (onTarget && !haveTarget) {
      runtime = Timing.get();
      showStatus("DriveToCargo Target acquired time:" + runtime);
      haveTarget = true;  
    }
    if(Timing.get()>10){
      runtime+=Timing.get();
      Autonomous.autoFailed=true;
      showStatus("DriveToCargo Failed to capture ball:" + runtime);
    }
    return false;
  }
  private void showStatus(String msg){
    SmartDashboard.putString("Status", msg);
    if(!msg.equals(last_msg))
      System.out.println(msg);
    last_msg=msg;
  }
}
