// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
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

  public static final int SHELF=8;
  public static final int GROUND=16;

  static int mode=MANUAL;

  static double wrist_incr=0.1;
  static double move_incr=0.005;

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
    SmartDashboard.putString("State","Disabled");
    m_arm.setHoldPose();
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.isRobotDisabled()){
       return;
    }
   
     // B button (claw open or close)
    if(m_controller.getBButtonPressed()){
      if(m_claw.clawOpen())
        m_claw.closeClaw();
      else
        m_claw.openClaw();
    }

    // A button (pickup mode)
    if(m_controller.getAButtonPressed()){ 
      SmartDashboard.putString("State","Pickup mode");
      switch(mode){
        default:
           mode=GETTING;
           break;
        case GETTING:
           mode=GETTING|GROUND;
           break;
        case GETTING|GROUND:
          mode=GETTING|SHELF;
          break;
        case GETTING|SHELF:
          mode=GETTING|GROUND;
          break;
      } 
    }
    
    // Y button (cone place mode)
    if(m_controller.getYButtonPressed()){
      switch(mode){
        default:
           break;
        case GETTING:
        case GETTING|SHELF:
        case GETTING|GROUND:
        case PLACING|CUBE:    
        case MANUAL:
           mode=HOLDING;
           break;
        case HOLDING:
          mode=PLACING|CONE;
          m_timer.reset();
          break;
        case PLACING|CONE:
          mode=HOLDING;
          break;
      } 
    }

    // X button (cube place mode)
    if(m_controller.getXButtonPressed()){
      switch(mode){
         default:
         break;
      case GETTING:
      case GETTING|SHELF:
      case GETTING|GROUND:
      case PLACING|CONE: 
      case MANUAL:
        mode=HOLDING;
        break;
      case HOLDING:
        mode=PLACING|CUBE;
        m_timer.reset();
        break;
      case PLACING|CUBE:
        mode=HOLDING;
        break;
      } 
    }
    // switch to manual mode if either triggers or dpad are used
    // trigger actions
    double left_trigger=m_controller.getLeftTriggerAxis();
    double right_trigger=m_controller.getRightTriggerAxis();
    if(left_trigger>0 || right_trigger>0){
      m_arm.setRotation(m_arm.getRotation() + (left_trigger-right_trigger) * wrist_incr);
      mode=MANUAL;
    }
    // dpad
    int pov=m_controller.getPOV();
    if(pov>=0){
      mode=MANUAL;
      double x=0;
      double y=0;
      switch(pov){
        case 0: 
          y+=move_incr;
          break;
        case 90:
          x-=move_incr;
          break;
        case 180:
          y-=move_incr;
          break;
        case 270:
          x+=move_incr;
          break;
      }    
      m_arm.setPose(pose[0]+x,pose[1]+y);
    }
    pose=m_arm.getTargetPosition();
    
    boolean rb=m_controller.getRightBumperPressed();
    boolean lb=m_controller.getLeftBumperPressed();
   
    // State Machine

    switch (mode){
      default:
      case GETTING:
        m_arm.setHoldPose();
      break;
      case MANUAL:
        if(rb)
          incrLevel();
        if(lb)
          decrLevel();
        SmartDashboard.putString("State","Manual Arm level:"+level);
        break;
      case HOLDING:
        if(rb)
          incrLevel();
        if(lb)
          decrLevel();
        m_arm.setHoldPose();
        SmartDashboard.putString("State","Holding Arm level:"+level);
        break;
      case PLACING|CONE:
        if(level==0){
          SmartDashboard.putString("State","Ground level place");
          m_arm.setBottomPose();
        }
        else if(level==1){
          SmartDashboard.putString("State","Placing CONE at Mid Level");
          m_arm.setMidConePose();
        }
        else{
          SmartDashboard.putString("State","Placing CONE at Top Level");
          m_arm.setTopConePose();
        }
        break;
      case PLACING|CUBE:
        if(level==0){
          SmartDashboard.putString("State","Ground level place");
          m_arm.setBottomPose();
        }
        else if(level==1){
          SmartDashboard.putString("State","Placing CUBE at Mid Level");
          m_arm.setMidCubePose();
        }
        else{
          SmartDashboard.putString("State","Placing CUBE at Top Level");
          m_arm.setTopCubePose();
        }
        break;
      case GETTING|SHELF:
        if(rb)
          mode=GETTING|GROUND;
        else{
          SmartDashboard.putString("State","Picking up from shelf");
          m_arm.setShelfPose();      
        }
        break;
      case GETTING|GROUND:
        if(lb)
          mode=GETTING|SHELF;
        else{
          SmartDashboard.putString("State","Picking up from ground");
          m_arm.setGroundPose();       
        }    
        break;
    }
  }
  void incrLevel(){
      level++;
      level = level > 2 ? 2 : level;
  }
  void decrLevel(){
    level--;
    level = level <= 0 ? 0 : level;
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
