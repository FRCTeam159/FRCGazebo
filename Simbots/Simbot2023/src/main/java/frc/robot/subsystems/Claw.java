// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gazebo.SimPiston;

public class Claw extends SubsystemBase {
  boolean m_clawopen=false;
  SimPiston m_piston = new SimPiston(0);
  public Claw() {
    closeClaw();
    m_piston.enable();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void openClaw() {
    m_clawopen=true;
    System.out.println("Claw openclaw");
    m_piston.forward();
  }

  public void closeClaw() {
    m_clawopen=false;
    System.out.println("Claw closeclaw");
    m_piston.reverse();
  }
  public boolean clawOpen(){
    return m_clawopen;
  }
}
