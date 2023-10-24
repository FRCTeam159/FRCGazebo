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

public class DriveToTarget extends CommandBase {
  double runtime;
  boolean haveTarget;
  boolean shooting;
  boolean starting;
  
  private final Targeting m_aim;
  private final Shooting m_shoot;
  private boolean started=false;
  boolean have_ball=false;
  private String last_msg;

  public DriveToTarget(Targeting targeting, Shooting shoot) {
    m_aim=targeting;
    m_shoot=shoot;
    
    addRequirements(shoot,targeting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    showStatus("DriveToTarget.init");
    started=false;

    have_ball=m_shoot.isBallCaptured();
    m_shoot.setIntakeHold();
    
    haveTarget=false;
    shooting=false;
    starting=false;
    runtime = 0;
    m_aim.setFrontTarget(true);
    m_aim.reset();
    Timing.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!haveTarget && m_aim.isFrontCamera()){
      if(!started)
        showStatus("DriveToTarget started " + Timing.get());
      started=true;
      m_aim.adjust();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_aim.disable();
    m_shoot.setShooterOff();
   // m_shoot.setIntakeOff();
    runtime += Timing.get();
    Autonomous.totalRuntime += runtime;
    if(Autonomous.autoFailed)
      showStatus("DriveToTarget.end - Auto failed");
    else
      showStatus("DriveToTarget.end " + runtime + " total time: " + Autonomous.totalRuntime);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!m_aim.frontCamera()|| !started)
      return false;
    if(!have_ball || Autonomous.autoFailed){
      if(!haveTarget)
        Autonomous.autoFailed=true;
      return true;
    }
    boolean onTarget = m_aim.onTarget();
    if (onTarget && !haveTarget) {
      runtime = Timing.get();
      showStatus("Target acquired time:" + runtime);
      Timing.reset();
      m_shoot.setShooterOn();
      haveTarget = true;  
    }
    if (!shooting && haveTarget && m_shoot.shooterReady(Shooting.kShootSpeed)) {
      m_shoot.setIntakeOn();
      //System.out.println("Shoot started:" + Timing.get() + " shooterspeeed: " + m_shoot.shooterSpeed());
      shooting=true;
      runtime+=Timing.get();
      Timing.reset();
    }
    if (shooting && Timing.get()> Shooting.kIntakeRunUpTime && !m_shoot.isBallCaptured()) {
      System.out.println("Done time:" + Timing.get());
      return true;
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
