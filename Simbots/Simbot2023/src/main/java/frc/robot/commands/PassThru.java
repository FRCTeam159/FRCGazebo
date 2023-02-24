// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import static frc.robot.Constants.*;


public class PassThru extends CommandBase {
   Arm m_arm;
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

  //double x_target=0.5;
   //double target[1]=0.325;  // must be set at x=0 point to avoid NaN
   double twist_90=Math.toRadians(-90);
   double twist_180=Math.toRadians(-180);
   double rotate_180=Math.toRadians(180);

   double target[]=new double[2];

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
  public PassThru(Arm arm, XboxController c) {
    m_arm=arm;
    m_controller=c;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target[0]=0.5;
    target[1]=0.325;
    m_timer.reset();
    m_timer.start();
    cnt=0;
    m_reversed=m_arm.getPosition()[0]<-0.25?true:false;
    System.out.println("PassThru.initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((state !=IDLE) && m_timer.get()>max_delay){
      System.out.println("state max delay exceeded - aborting");
      m_arm.setInitPose();
      state=END;
    }
    d=m_arm.getPosition();
    double x=d[0];
    double y=d[1];
    cnt++;
    switch(state){
      default:
      case IDLE:
        if(kTestMode && m_controller.getRightBumperPressed()){
          m_reversed=x<-0.25?true:false;
          System.out.println("PassThru: starting reversed:"+m_reversed);
          target[0]=reversed()?-0.5:0.5;
          m_arm.setPose(target[0],target[1]);
          resetTimer(default_delay);
          state=STARTED;
        }
        break;
      case STARTED:
        if(Math.abs(x-target[0])<maxXerr && Math.abs(y-target[1])<maxYerr){
          System.out.format("PassThru: reached start position x:%-1.2f y:%-1.2f\n",x, y);
          m_arm.setTwist(twist_90);
          resetTimer(default_delay);
          state=TWIST_90;
        }
        break;
      case TWIST_90:
        if(Math.abs(m_arm.getTwist()-twist_90)<maxRotationError){
          System.out.println("PassThru: reached twist angle -90");
          rotation=reversed()?0:rotate_180;
          m_arm.setRotation(rotation);
          state=ROTATE_180;
          resetTimer(default_delay);
        }
        break;
        case ROTATE_180: 
        if(Math.abs(m_arm.getRotation()-rotation)<maxRotationError){
          System.out.format("PassThru: reached rotation angle %-3.1f\n",Math.toDegrees(rotation));
          target[0]=reversed()?0.5:-0.5;
          m_arm.setPose(target[0],target[1]);
          state=MOVE_X;
          resetTimer(2*default_delay);
        }
        break;
      case MOVE_X:
        if(Math.abs(x-target[0])<maxXerr && Math.abs(y-target[1])<maxYerr){
          System.out.println("PassThru: reached end position x:"+target[0]);
          resetTimer(default_delay);
          twist=reversed()?0:twist_180;
          m_arm.setTwist(twist);
          state=TWIST_180;
        }
        break;
      case TWIST_180:
        if(Math.abs(m_arm.getTwist()-twist)<maxRotationError){
          System.out.format("PassThru: end x:%-1.2f y:%-1.2f rot:%-3.1f twist:%-3.1f\n",
             x,y,Math.toDegrees(m_arm.getRotation()),Math.toDegrees(m_arm.getTwist()));
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
