// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// climbing routine
// 1) position bot in back of second bar facing outward (vert,diag arms retracted)
// 2) extent vert arms to max ht
// 3) drive forward so hooks are over bar
// 4) retract vert arms to min ht
// 5) extend diag arms (should be over bar)
// 6) extend vert arms to max ht (weight should transfer to diag arms)
// 7) robot should tilt back so vert arms behind 2nd bar
// 8) when that happens, retract vert bars to <= 1/2 ht
// 9) then quickly extend vert bars to full ht
// 10) robot base should swing out towards field
// 11) when at max swing pull in diag arms (should slide off top of first bar)
// 12) robot has reached 3rd bar at this point with vert arm extended and diag arms in
// 13) repeat 4-11 for top bar
 

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.objects.Piston;
import frc.robot.objects.SparkMotor;

public class Climber extends SubsystemBase implements Constants{
  /** Creates a new Climber. */
  SparkMotor m_lifter=new SparkMotor(CLIMBER);
  Piston m_piston=new Piston(1);
  boolean m_arms_out;
  double setval=0;
  final double adjust_delta_slow=0.005;
  final double adjust_delta_fast=0.05;
  double adjust_delta=adjust_delta_slow;

  double max_setpoint=1.05;
  double min_setpoint=0;
  final PIDController m_controller=new PIDController(3,0.6,0.1);
  
  public Climber() {
    SmartDashboard.putBoolean("Arms out",m_arms_out);
    SmartDashboard.putNumber("Lifter",0);
    m_lifter.setDistancePerRotation(1);
    m_controller.setTolerance(0.05, 0.05);
    m_controller.setIntegratorRange(-4, 4.0);
    m_lifter.setInverted();
    m_arms_out=false;
    reset();
  }

  public void adjustSetpoint(double f){
    setval+=f*adjust_delta;
    setval=setval<min_setpoint?min_setpoint:setval;
    setval=setval>max_setpoint?max_setpoint:setval;
  }
  public void init(){
    m_lifter.reset();
    m_lifter.enable();
    armsIn();
  }
  public void enable(){
    m_lifter.enable();
    m_piston.enable();
  }

  public void reset(){
    m_lifter.reset();
    setval=0.0;
    m_controller.reset();
    //setLifterDown(0.1);
  }
  public void disable(){
    m_lifter.disable();
    m_piston.disable();
  }
 
  public void setLifterUp(){
    setval=1;
  }
  public void setLifterDown(){
    setval=0.0;
  }

  public void setLifterUp(double f){
    adjust_delta=adjust_delta_slow;
    adjustSetpoint(f);
  }
  public void setLifterDown(double f){
    adjust_delta=adjust_delta_slow;
    adjustSetpoint(-f);
  }

  public void armsOut(){
    m_arms_out=true;
    m_piston.setForward();
  }
  public void armsIn(){
    m_arms_out=false;
    m_piston.setReverse();
  }

  @Override
  public void periodic() {
    double distance=-m_lifter.getDistance();
    SmartDashboard.putNumber("Lifter",distance);
    SmartDashboard.putNumber("Setval",setval);
    SmartDashboard.putBoolean("Arms out",m_arms_out);
    double correction=m_controller.calculate(distance,setval);
    m_lifter.set(-correction);
  }
}
