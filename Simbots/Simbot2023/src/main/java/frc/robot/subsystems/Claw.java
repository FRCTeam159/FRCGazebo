// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import gazebo.SimPiston;

public class Claw extends SubsystemBase {
  boolean m_clawopen=false;
  SimPiston m_piston = new SimPiston(0);
  
  public Claw() {
    m_piston.enable();
    closeClaw();
  }

  @Override
  public void periodic() {
    if(Robot.isRobotDisabled())
      closeClaw();
  }
  public void openClaw() {
    m_clawopen=true;
    System.out.println("Claw openclaw");
    m_piston.forward();
  }
  public void closeClaw() {
    m_clawopen=false;
    if(!Robot.isRobotDisabled())
    System.out.println("Claw closeclaw");
    m_piston.reverse();
  }
  public boolean clawOpen(){
    return m_clawopen;
  }
}
