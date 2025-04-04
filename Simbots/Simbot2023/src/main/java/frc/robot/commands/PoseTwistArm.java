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

public class PoseTwistArm extends CommandBase {
  Arm m_arm;
  Claw m_claw;

  XboxController m_controller;

  double angle=0;

  Timer m_timer=new Timer();
  
  static int level=0;

  public static final int TEST=0;
  public static final int PLACING=1;
  public static final int GETTING=2;
  public static final int HOLDING=4;

  public static final int EMPTY=0;
  public static final int CUBE=8;
  public static final int CONE=16;

  static int mode=TEST;

  static double wrist_incr=0.1;
  static double move_incr=0.01;

  public static final int FIXED=0;
  public static final int ROTATE=1;
  public static final int TWIST=2;
  public static final int MOVE1=3;
  public static final int MOVE2=4;

  static int turn_mode=ROTATE;

  String test_str="Wrist Rotate";

  double pose[]=new double[2];
  
  public PoseTwistArm(Arm arm, Claw claw, XboxController controller) {
    m_arm = arm;
    m_claw=claw;
    m_controller=controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArmTest.initialize");
    SmartDashboard.putString("State","Driving");
    m_arm.setInitPose();
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // button actions

    if(m_controller.getBButtonPressed()){
      if(m_claw.clawOpen())
        m_claw.closeClaw();
      else
        m_claw.openClaw();
    }

    if(m_controller.getAButtonPressed()){ 
      setTestMode();
      switch (turn_mode){
        default:
        case ROTATE:
          test_str="Wrist Twist";
          turn_mode=TWIST;
          break;
        case TWIST:
          test_str="Move X";
          turn_mode=MOVE1;
          break;
        case MOVE1:
          test_str="Move Y";
          turn_mode=MOVE2;
          break;
        case MOVE2:
          test_str="Wrist Rotate";
          turn_mode=ROTATE;
          break;
      }
    }
    
    if(m_controller.getYButtonPressed()){
      clrTestMode();
      switch(mode){
        default:
        //   mode=GETTING|CONE;
           break;
        case TEST:
           mode=HOLDING;
           break;
        case HOLDING:
          mode=PLACING|CONE;
          m_timer.reset();
          break;
        // case GETTING|CONE:
        //    mode=PLACING|CONE;
        //    break;
        case PLACING|CONE:
          mode=HOLDING;
          break;
      } 
    }
    if(m_controller.getXButtonPressed()){
      clrTestMode();
      switch(mode){
         default:
         break;
      case TEST:
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
    // trigger actions
    double left_trigger=m_controller.getLeftTriggerAxis();
    double right_trigger=m_controller.getRightTriggerAxis();
    pose=m_arm.getTargetPosition();
    switch (turn_mode) {
      case FIXED:
        break;
      case ROTATE:
        if (right_trigger > 0) 
          m_arm.setRotation(m_arm.getRotation() - right_trigger * wrist_incr);
        if (left_trigger > 0) 
          m_arm.setRotation(m_arm.getRotation() + left_trigger * wrist_incr);
        break;
      case TWIST:
        if (right_trigger > 0) 
          m_arm.setTwist(m_arm.getTwist() - right_trigger * wrist_incr);
        if (left_trigger > 0) 
          m_arm.setTwist( m_arm.getTwist() + left_trigger * wrist_incr);     
        break;
      case MOVE1:
        if (right_trigger > 0.1) {
          pose[0]=m_arm.getXTarget()+right_trigger * move_incr;
          m_arm.setPose(pose[0],pose[1]);
        }
        if (left_trigger > 0.1){
          pose[0]=m_arm.getXTarget()-left_trigger * move_incr;
          m_arm.setPose(pose[0],pose[1]);
        }
        break; 
      case MOVE2:
        if (right_trigger > 0.1) {
          pose[1]=m_arm.getXTarget()+right_trigger * move_incr;
          m_arm.setPose(pose[0],pose[1]);
        }
        if (left_trigger > 0.1) {
          pose[1]=m_arm.getYTarget()-left_trigger * move_incr;
          m_arm.setPose(pose[0],pose[1]);
        }
        break;       
    }
    
    // state machine
    switch (mode){
      default:
      case TEST:
        SmartDashboard.putString("State","Test "+test_str);
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
          m_arm.setGroundPose();
          m_arm.setGroundPose();
        }
        else if(level==1){
          m_arm.setMidConePose();
          m_arm.setMidConePose();
        }
        else{
          m_arm.setTopConePose();
          m_arm.setTopConePose();
        }
        SmartDashboard.putString("State","Placing CONE at Level:"+level);
        break;
      case PLACING|CUBE:
        if(level==0){
          m_arm.setGroundPose();
          m_arm.setGroundPose();
        }
        else if(level==1){
          m_arm.setMidCubePose();
          m_arm.setMidCubePose();
        }
        else{
          m_arm.setTopCubePose();
          m_arm.setTopCubePose();
        }
        SmartDashboard.putString("State","Placing CUBE at Level:"+level);
        break;
      case GETTING|CONE:
        m_arm.setShelfPose();
        m_arm.setShelfPose();
        SmartDashboard.putString("State","Getting CONE from shelf");
        break;
      case GETTING|CUBE:
        SmartDashboard.putString("State","Getting CUBE from ground");
        m_arm.setGroundPose();
        m_arm.setGroundPose();
        break;
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
  void setTestMode(){
    mode=TEST;
    kTestMode=true;
  }
  void clrTestMode(){
    kTestMode=false;
  }
}
