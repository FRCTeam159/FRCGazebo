// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import static frc.robot.Constants.*;

public class PoseDualArm extends CommandBase {
  Arm m_arm;
  Claw m_claw;

  XboxController m_controller;

  double angle=0;

  Timer m_timer=new Timer();
  
  static int level=0;

  public static final int MANUAL=0;
  public static final int PLACING=1;
  public static final int GETTING=2;
  public static final int HOLDING=4;

  public static final int EMPTY=0;
  public static final int CUBE=8;
  public static final int CONE=16;

  static int mode=MANUAL;

  static double wrist_incr=0.1;
  static double move_incr=0.01;

  double pose[]=new double[2];
  
  public PoseDualArm(Arm arm, Claw claw, XboxController controller) {
    m_arm = arm;
    m_claw=claw;
    m_controller=controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PoseDualArm.initialize");
    SmartDashboard.putString("State","Driving");
    m_arm.setInitPose();
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getBButtonPressed()){
      if(m_claw.clawOpen())
        m_claw.closeClaw();
      else
        m_claw.openClaw();
    }

    if(m_controller.getAButtonPressed()){ 
      SmartDashboard.putString("State","Getting CONE from shelf");
      if(mode==(GETTING|CONE))
        mode=HOLDING;
      else
        mode=GETTING|CONE;
    }
    
    if(m_controller.getYButtonPressed()){
      switch(mode){
        default:
           break;
        case GETTING|CONE:
        case MANUAL:
           level=0;
           mode=HOLDING;
           break;
        case HOLDING:
          mode=PLACING|CONE;
          m_timer.reset();
          break;
        case PLACING|CONE:
          level=0;
          mode=HOLDING;
          break;
      } 
    }
    if(m_controller.getXButtonPressed()){
      switch(mode){
         default:
         break;
      case GETTING|CONE:
      case MANUAL:
        level=0;
        mode=HOLDING;
        break;
      case HOLDING:
        mode=PLACING|CUBE;
        m_timer.reset();
        break;
      case PLACING|CUBE:
        mode=HOLDING;
        level=0;
        break;
      } 
    }
    // trigger actions
    double left_trigger=m_controller.getLeftTriggerAxis();
    double right_trigger=m_controller.getRightTriggerAxis();
    if (right_trigger > 0) 
      m_arm.setRotation(m_arm.getRotation() - right_trigger * wrist_incr);
    if (left_trigger > 0) 
      m_arm.setRotation(m_arm.getRotation() + left_trigger * wrist_incr);
 
    pose=m_arm.getTargetPosition();
    
    // state machine
    switch (mode){
      default:
      case MANUAL:
        if (right_trigger > 0) 
          m_arm.setRotation(m_arm.getRotation() - right_trigger * wrist_incr);
        if (left_trigger > 0) 
          m_arm.setRotation(m_arm.getRotation() + left_trigger * wrist_incr);
        SmartDashboard.putString("State","Arm move using POV");
        break;
      case HOLDING:
        m_arm.setInitPose();
        if (m_controller.getRightBumperPressed()) {
          level++;
          level = level > 2 ? 2 : level;
        }
        if (m_controller.getLeftBumperPressed()) {
          level--;
          level = level <= 0 ? 0 : level;
        } 
        SmartDashboard.putString("State","Holding level:"+level);
        break;
      case PLACING|CONE:
        if(level==0){
          SmartDashboard.putString("State","Ground level plce/pickup");
          m_arm.setGroundPose();
          m_arm.setGroundPose();
        }
        else if(level==1){
          SmartDashboard.putString("State","Placing CONE at Mid Level");
          m_arm.setMidConePose();
          m_arm.setMidConePose();
        }
        else{
          SmartDashboard.putString("State","Placing CONE at Top Level");
          m_arm.setTopConePose();
          m_arm.setTopConePose();
        }
        break;
      case PLACING|CUBE:
        if(level==0){
          SmartDashboard.putString("State","Ground level place or pickup");
          m_arm.setGroundPose();
          m_arm.setGroundPose();
        }
        else if(level==1){
          SmartDashboard.putString("State","Placing CUBE at Mid Level");
          m_arm.setMidCubePose();
          m_arm.setMidCubePose();
        }
        else{
          SmartDashboard.putString("State","Placing CUBE at Top Level");
          m_arm.setTopCubePose();
          m_arm.setTopCubePose();
        }
        break;
      case GETTING|CONE:
        m_arm.setShelfPose();
        m_arm.setShelfPose();
        SmartDashboard.putString("State","Getting CONE from shelf");
        break;
      // case GETTING|CUBE:
      //   SmartDashboard.putString("State","Getting CUBE from ground");
      //   m_arm.setGroundPose();
      //   m_arm.setGroundPose();
      //   break;
    }
    int pov=m_controller.getPOV();
    if(pov>=0){
      mode=MANUAL;
      double x=0;
      double y=0;
      switch(pov){
        case 0: 
          y+=move_incr;
          break;
        case 45:
          y+=0.5*move_incr;
          x-=0.5*move_incr;
          break;
        case 90:
          x-=move_incr;
          break;
        case 180:
          y-=move_incr;
          break;
        case 225:
          y-=0.5*move_incr;
          x+=0.5*move_incr;
          break;
        case 270:
          x+=move_incr;
          break;
        case 315:
          y+=0.5*move_incr;
          x+=0.5*move_incr;
        break;
      }
      // double a=move_incr*Math.toRadians((double)pov);
      // double x=move_incr*Math.cos(a);
      // double y=move_incr*Math.sin(a);
      // pose[0]+=x;
      // pose[1]+=y;
      //System.out.println("pov="+pov);
      m_arm.setPose(pose[0]+x,pose[1]+y);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kTestMode=false;
    System.out.println("ArmTest.end:"+interrupted);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
