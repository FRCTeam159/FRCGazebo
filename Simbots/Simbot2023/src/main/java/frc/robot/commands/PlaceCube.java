// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PlaceCube extends CommandBase {
  enum placeState {
    start, platform,finish
  };
  Arm m_arm;
  Claw m_claw;
  int m_lvl=2;
  Timer m_timer=new Timer();
  placeState state=placeState.start;
  /** Creates a new PlaceCube. 
 * @param i
 * @param m_claw
 * @param m_arm*/
  public PlaceCube(int i, Arm arm, Claw claw) {
    m_arm=arm;
    m_claw=claw;
    m_lvl=i;
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PlaceCube.initialize");
    switch(m_lvl){
      case 0:
        m_arm.setGroundPose();
        break;
      case 1:
        m_arm.setMidCubePose();
        break;
      case 2:
        m_arm.setTopCubePose();
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PlaceCube.end");
    //m_claw.openClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch(state){
      case start:
        if(m_arm.onTarget()){
          m_timer.reset();
          state=placeState.platform;
          m_claw.openClaw();
        }
      break;
      case platform:
      if(m_timer.get()>1.0){
        m_arm.setInitPose();
        state=placeState.finish;
      }
      break;
      case finish:
        if(m_arm.onTarget())
          return true;
      break;
    }
    return false;
  }
}
