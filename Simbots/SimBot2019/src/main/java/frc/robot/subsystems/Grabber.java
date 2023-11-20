/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import gazebo.SimEncMotor;
import gazebo.SimPiston;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.GrabberCommands;

/**
 * Add your docs here.
 */
public class Grabber extends SubsystemBase {
  //private SimEncMotor armMover = new SimEncMotor(RobotMap.ARM_SERVO);
  private SimEncMotor grabberMotor = new SimEncMotor(RobotMap.GRABBER_MOTOR);
  SimPiston grabPneumatic = new SimPiston(2);
  SimPiston tiltPneumatic = new SimPiston(1);

  private double spinCW = 1.0;
  private double spinCCW = -1.0;
  private boolean clawOpen = false;
  private boolean inputActive=false;
  private boolean ejectActive=false;

  boolean tilted=false;

  public Grabber(){
    grabPneumatic.enable();
    tiltPneumatic.enable();
    grabberMotor.enable();
    log();
  }
 
  public void initDefaultCommand() {
    setDefaultCommand(new GrabberCommands());
  }
  public void init(){
    tilt(false);
    openClaw();
  }
  public boolean isTilted(){
    return tilted;
  }
  public boolean isClawOpen(){
    return clawOpen;
  }
  public void closeClaw(){
    grabPneumatic.set(-5);
    clawOpen = false;
    log();
  }
  public void openClaw(){
    grabPneumatic.set(5);
    clawOpen = true;
    log();
  }
  public void eject(){
    if(Robot.hatchMode)
      grabberMotor.set(spinCW);
    else
      grabberMotor.set(spinCCW);
    ejectActive=true;
    log();
  }
  public void grab(){
    if(Robot.hatchMode)
      grabberMotor.set(spinCCW);
    else
      grabberMotor.set(spinCW);
    inputActive=true;
    log();
  }
  public void hold(){
    grabberMotor.set(0);
    inputActive=false;
    ejectActive=false;
    log();
  }
  public void dropGrabber(){
    tilt(true);
  }
  public void tilt(boolean forward){
    if(forward){
      System.out.println("Grabber.tilt(forward)");
      tiltPneumatic.set(-1);
      tilted=false;
    }
    else{
      System.out.println("Grabber.tilt(back)");
      tiltPneumatic.set(1);
      tilted=true;
    }
    log();
  }
  void log(){
    SmartDashboard.putBoolean("Input On", inputActive);
    SmartDashboard.putBoolean("Eject On", ejectActive);
    SmartDashboard.putBoolean("Claw Open", clawOpen);
    SmartDashboard.putBoolean("Grabber Tilted", tilted);
    SmartDashboard.putBoolean("HatchMode", Robot.hatchMode);
    SmartDashboard.putBoolean("CargoMode", Robot.cargoMode);
  }
}
