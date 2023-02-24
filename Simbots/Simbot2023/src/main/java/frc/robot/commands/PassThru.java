// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class PassThru extends CommandBase {
   Arm m_arm;
   Wrist m_wrist;
   XboxController m_controller;

   final static int IDLE=0;
   final static int STARTED=1;
   final static int SET_XY=2;
   final static int MOVE_X=3;
   final static int ROTATE_180=4;
   final static int TWIST_90=5;
   final static int TWIST_180=6;
   final static int END=7;

   boolean m_reversed=false;

   int cnt=0;

   double x_target=0.5;
   double y_target=0.325;  // must be set at x=0 point to avoid NaN
   double twist_90=Math.toRadians(-90);
   double twist_180=Math.toRadians(-180);
   double rotate_180=Math.toRadians(180);

   double maxRotationError=Math.toRadians(5);
   double maxXerr=0.1;
   double maxYerr=0.05;

   static int state=IDLE;

   double default_delay=30;
   double max_delay=default_delay;

   double d[]=new double[2];
   double rotation=0;
   double twist=0;

   Timer m_timer=new Timer();
  /** Creates a new PassThru. */
  public PassThru(Wrist w, Arm a, XboxController c) {
    m_wrist=w;
    m_arm=a;
    m_controller=c;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    cnt=0;
    m_reversed=m_arm.getX()<-0.25?true:false;
    System.out.println("PassThru.initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((state !=IDLE) && m_timer.get()>max_delay){
      System.out.println("state max delay exceeded - aborting");
      m_arm.setInitPose();
      m_wrist.setInitialPose();
      state=END;
    }
    // if(m_arm.testMode() && m_controller.getLeftBumperPressed()){
    //   System.out.println("Passthru apborted by user");
    //   m_arm.setHoldingPose();
    //   m_wrist.setHoldingPose();
    //   state=IDLE;
    // }
   
    // if((cnt%50)==0){
    //   System.out.println("PassThru state="+state);  
    // }
    cnt++;
    switch(state){
      default:
      case IDLE:
        if(m_arm.testMode() && m_controller.getRightBumperPressed()){
          m_reversed=m_arm.getX()<-0.25?true:false;
          System.out.println("PassThru: starting reversed:"+m_reversed);
          x_target=reversed()?-0.5:0.5;
          m_arm.setPose(x_target,y_target);
          resetTimer(default_delay);
          state=STARTED;
        }
        break;
      case STARTED:
        if(Math.abs(m_arm.getX()-x_target)<maxXerr && Math.abs(m_arm.getY()-y_target)<maxYerr){
          System.out.format("PassThru: reached start position x:%-1.2f y:%-1.2f\n",m_arm.getX(), m_arm.getY());
          m_wrist.setTwist(twist_90);
          resetTimer(default_delay);
          state=TWIST_90;
        }
        break;
      case TWIST_90:
        if(Math.abs(m_wrist.getTwist()-twist_90)<maxRotationError){
          System.out.println("PassThru: reached twist angle -90");
          rotation=reversed()?0:rotate_180;
          m_wrist.setRotation(rotation);
          state=ROTATE_180;
          resetTimer(default_delay);
        }
        break;
        case ROTATE_180: 
        if(Math.abs(m_wrist.getRotation()-rotation)<maxRotationError){
          System.out.format("PassThru: reached rotation angle %-3.1f\n",Math.toDegrees(rotation));
          x_target=reversed()?0.5:-0.5;
          m_arm.setPose(x_target,y_target);
          state=MOVE_X;
          resetTimer(2*default_delay);
        }
        break;
      case MOVE_X:
        if(Math.abs(m_arm.getX()-x_target)<maxXerr && Math.abs(m_arm.getY()-y_target)<maxYerr){
          System.out.println("PassThru: reached end position x:"+x_target);
          resetTimer(default_delay);
          twist=reversed()?0:twist_180;
          m_wrist.setTwist(twist);
          state=TWIST_180;
        }
        break;
      case TWIST_180:
        if(Math.abs(m_wrist.getTwist()-twist)<maxRotationError){
          System.out.format("PassThru: end x:%-1.2f y:%-1.2f rot:%-3.1f twist:%-3.1f\n",
             m_arm.getX(),m_arm.getY(),Math.toDegrees(m_wrist.getRotation()),Math.toDegrees(m_wrist.getTwist()));
          state=END;
        }
        break;
      case END:
        System.out.println("passthru complete");
        state=IDLE;
        break;
    }
  }

  boolean reversed(){
    return m_reversed;
  }
  void resetTimer(double d){
    m_timer.reset();
    max_delay=d;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PassThru.end()");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
