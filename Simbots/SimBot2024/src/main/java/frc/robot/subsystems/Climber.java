// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import gazebo.SimEncMotor;
import gazebo.SimSwitch;

public class Climber extends SubsystemBase implements Constants{
  private final SimEncMotor m_climberMotor=new SimEncMotor(CLIMBER);
  boolean m_up=true;
  boolean m_climbing=false;
  double m_climb_increment=0.1;
  Timer m_timer = new Timer();
  boolean m_debug=true;
  double m_position=0;
  static double m_max=7;
  static double m_min=-12;
  double m_disabled_backdrive=1.5;
  static double m_climb_target=m_max; // highest position (inches)

  PIDController m_pid=new PIDController(0.3,0.05,0);
  private SimSwitch climbLimit = new SimSwitch(CLIMB_LIMIT);
  /** Creates a new Climber. */
  public Climber() {
    m_timer.start();
    m_climberMotor.enable();
    m_climberMotor.setDistancePerRotation(18/0.0636);
    m_pid.setIntegratorRange(0, 0.6);
    climbLimit.enable();
  }
  
  @Override
  public void periodic() {
    double corr=0;
    if(Robot.initialized && Robot.disabled){
      corr=m_disabled_backdrive;
    }
    else{
      m_pid.setSetpoint(m_position);
      corr=m_pid.calculate(position(),m_position);
    }
    m_climberMotor.set(corr);
    String s=String.format("P:%-3.1f T:%-3.1f C:%-3.2f",position(),m_position,corr);
    SmartDashboard.putString("Climber",s);
    log();
  }

  public double position(){
    return m_climberMotor.getDistance();
  }

  public void reset(){
    m_position=0;
  }
  public void enable(){
    m_climbing=true;
  }
  public void disable(){
     m_climbing=false;
  }
  public void climbUp(){
    m_position+=m_climb_increment;
    m_position=m_position>m_max?m_max:m_position;
  }
  public void climbDown(){
    m_position-=m_climb_increment;
    m_position=m_position<m_min?m_min:m_position;
  }

  public void hookChain(){
    m_position=m_min;
 }
 public void climbToTarget(){
    m_position=m_climb_target;
 }
  public boolean atBottom() {
    return climbLimit.lowLimit();
  }

  public boolean atTop() {
    return climbLimit.highLimit();
  }

  void log(){
    SmartDashboard.putBoolean("ClimberTop",atTop());
    SmartDashboard.putBoolean("ClimberBot",atBottom());
  }
}
