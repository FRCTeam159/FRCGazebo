// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.OneArm;

public class PoseOneArm extends CommandBase {
  public OneArm m_arm;
  public XboxController m_controller;
  public Claw m_claw;
  public enum m{
    pickup,
    hold,
    drop,
    eject,
    none
  };
  public enum a{
    arm,
    wrist
  };
  public m claw_mode = m.none;
  public a arm_mode = a.arm;

  public Timer tim = new Timer();
  public double k = 0; //set to 113;
  /** Creates a new PoseArm. */
  public PoseOneArm(OneArm arm, XboxController controller, Claw claw) {
    m_arm = arm;
    m_controller = controller;
    m_claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Arm.posHolding();
    tim.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getYButtonPressed()){
      if(arm_mode==a.arm)
        arm_mode=a.wrist;
      else if(arm_mode==a.wrist)
        arm_mode=a.arm;
    }
    double up = m_controller.getRightTriggerAxis();
    double down = m_controller.getLeftTriggerAxis();
    if(arm_mode==a.arm){
      if(up>0)
        m_arm.setArm(0.01*up);
      if(down>0)
        m_arm.setArm(-0.01*down);
    }
    else{
      if(up>0)
        m_arm.setWrist(0.01*up);
      if(down>0)
        m_arm.setWrist(-0.01*down);
    }

    if(m_controller.getBButtonPressed()){
      if(m_claw.clawOpen())
        m_claw.closeClaw();
      else
        m_claw.openClaw();
    }
  }

 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
