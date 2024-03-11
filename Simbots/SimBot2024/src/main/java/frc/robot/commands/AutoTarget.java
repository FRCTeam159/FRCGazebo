// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagDetector;
import frc.robot.subsystems.TargetMgr;
import objects.AprilTag;

public class AutoTarget extends Command {
  Arm m_arm;
  Drivetrain m_drive;
  PIDController turnPID = new PIDController(1, 0, 0);
  PIDController anglePID = new PIDController(1, 0, 0);
  AprilTag[] tags;
  Timer m_timer=new Timer();
  boolean have_tags=false;
  static boolean debug=false;
  

  /** Creates a new AutoTarget. 
 * @param drive 
 * @param arm */
  public AutoTarget(Arm arm, Drivetrain drive) {
    m_arm=arm;
    m_drive=drive;
    addRequirements(drive);
    turnPID.setSetpoint(TargetMgr.kHorizOffset);
    anglePID.setSetpoint(TargetMgr.kVertOffset); // y offset if at speaker steps
    anglePID.setTolerance(0.05,0.02);
    turnPID.setTolerance(.05,0.01);
    m_timer.start();
    tags=null;
   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Autonomous.log("AutoTarget.init");
    TagDetector.setTargeting(true);
    tags=TagDetector.getTags();
    m_timer.reset();
    have_tags=false;
    anglePID.reset();
    turnPID.reset();
    Autonomous.setOnTarget(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tags=TagDetector.getTags();
    if(tags==null){
      have_tags=false;
      return;
    }
    if(tags.length==1 && tags[0].getTagId() !=7 && tags[0].getTagId()!=4){
      have_tags=false;
      return;
    }
    have_tags=true;
    
    TargetMgr.setBestTarget(tags);
    AprilTag target=tags[TargetMgr.kBestTarget];

    double y=target.getY();
    double z=target.getZ();
    double acorr=anglePID.calculate(z);
    double a=m_arm.getTargetAngle();
    m_arm.setTargetAngle(a+acorr); // TODO: adjust angle based on distance (target.getX())

    double rcorr=turnPID.calculate(y);
    m_drive.drive(0, 0,-rcorr,false);
    if(debug)
      System.out.println(target);
  }

  public double getZOffset(double x) {
    // X       Z
    // 1.67    -0.2
    // 2.5     -0.04
    // 3.09    -0
    return -0.2 + (x-1.67) * ((0 - -0.2) / (3.09 - 1.67));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Autonomous.log("AutoTarget.end");
    TagDetector.setTargeting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Autonomous.okToRun())
      return true;
    if(!have_tags && m_timer.get()>2){
      Autonomous.log("Autotarget.end - timeout expired");
      return true;
    }
    if(anglePID.atSetpoint() && turnPID.atSetpoint()){
      Autonomous.setOnTarget(true);
      Autonomous.log("Autotarget.end - on target");
      return true;
    }
    return false;
  }
}
