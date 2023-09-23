// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PlaceCube extends CommandBase {

  public static final double[] kmid =  {0.2,0.7,Math.toRadians(-20)}; // middle position to avoid hitting grid

  enum placeState {
    start, mid, top,finish
  };
  Arm m_arm;
  Claw m_claw;
  int m_lvl=2; // default
  Timer m_timer=new Timer();
  placeState state=placeState.start;
  /** Creates a new PlaceCube. 
 * @param i
 * @param m_claw
 * @param m_arm
 * */
  public PlaceCube(int lvl, Arm arm, Claw claw) {
    m_arm=arm;
    m_claw=claw;
    m_lvl=lvl;
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.closeClaw();
    System.out.println("PlaceCube.initialize");
    state=placeState.start;
    switch(m_lvl){
      case 0:
        m_arm.setBottomPose();
        break;
      case 1:
      case 2:
        m_arm.setPose(kmid);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PlaceCube.end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch(state){
      case start:
        if(m_arm.onTarget()){ // first target
          switch(m_lvl){
            case 0:
              m_timer.reset();
              state=placeState.top;
              m_claw.openClaw();
              break;
            case 1:
              m_arm.setMidCubePose();
              state=placeState.mid;
              break;
            default:
            case 2:
              m_arm.setTopCubePose();
              state=placeState.mid;
              break;
          }
        }
        break;
      case mid:
        if(m_arm.onTarget()){ // second (final) target
          m_timer.reset();
          state=placeState.top;
          m_claw.openClaw();
        }
        break;
      case top:
        if(m_timer.get()>1.0){ // open claw with enough time to drop cube
          m_arm.setHoldPose(); // then go to holding pose
          state=placeState.finish;
        }
        break;
      case finish:
        if(m_arm.onTarget()) // wait for arm to get to holding pose
          return true;
      break;
    }
    return false;
  }
}
