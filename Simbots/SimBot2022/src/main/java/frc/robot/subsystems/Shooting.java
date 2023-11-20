// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.objects.SparkMotor;
import gazebo.SimContact;
import utils.MathUtils;

public class Shooting extends SubsystemBase implements Constants {
  /** Creates a new Intake. */
  private SparkMotor intake;
  private SparkMotor shoot;
  private SimContact limit_switch;
  private boolean intake_is_on = false;
  private boolean intake_is_off = true;
  private boolean intake_is_holding = false;
  private boolean shooter_is_on = false;
  private boolean ball_captured = false;
  private double shooter_err_tol = 0.1;
  private double shooter_err_vel_tol = 0.02;
  private double intake_err_tol = 0.7;
  private double intake_err_vel_tol = 0.1;

  public static double kIntakeSpeed = 8;
  public static double kIntakeHold = -1;
  public static double kShootSpeed = 17.5;
  public static double kShootHold = 0;
  public static double kShooterRunUpTime = 3;
  public static double kShooterRunDownTime = 5;
  public static double kIntakeRunUpTime = 1.5;
  public static double kIntakeRunDownTime = 1;
  public static double kSpinbackTime = 1;

  private final static PIDController m_shooter_controller = new PIDController(0.1, 0.0, 0);
  private final static PIDController m_intake_controller = new PIDController(0.1, 0.0, 0);


  private boolean shooter_ramp = true;
  private boolean intake_ramp = true;

  private Ramp shooterRamp=new Ramp("shooter");
  private Ramp intakeRamp=new Ramp("intake");
  
  public Shooting() {
    intake = new SparkMotor(INTAKE);
    shoot = new SparkMotor(SHOOTER);

    m_shooter_controller.setTolerance(shooter_err_tol, shooter_err_vel_tol);
    m_intake_controller.setTolerance(intake_err_tol, intake_err_vel_tol);

    shoot.setInverted();
    
    limit_switch = new SimContact(0);
    intake.enable();
    shoot.enable();
    limit_switch.enable();
    setIntakeHold();
    setShooterOff();
    
    SmartDashboard.putNumber("Shooter speed", 0);
    SmartDashboard.putNumber("Target speed", kShootSpeed);
    SmartDashboard.putNumber("Intake speed", 0);
    SmartDashboard.putBoolean("Ball captured", ball_captured);
    SmartDashboard.putBoolean("Intake on", intake_is_on);
    SmartDashboard.putBoolean("Shooter on", shooter_is_on);
  }

  public void setShooterOn() {
    if(shooter_ramp && !shooter_is_on)
      shooterRamp.config(kShooterRunUpTime,kShootHold,kShootSpeed,0.5);      
    shooter_is_on = true;
  }

  public void setShooterOff() {
    if(shooter_ramp && shooter_is_on)
      shooterRamp.config(kShooterRunDownTime,kShootSpeed,kShootHold,0.5);
    shooter_is_on = false;
  }

  public void setIntakeOn() {
   
    if(intake_ramp && !intake_is_on)
      intakeRamp.config(kIntakeRunUpTime,kIntakeHold,kIntakeSpeed,0.5);
    intake_is_on = true;
    intake_is_holding=false;
    intake_is_off=false;
  }

  public void setIntakeOff() {
    if(intake_ramp && intake_is_on)
      intakeRamp.config(kSpinbackTime,kIntakeSpeed,0,0.5);
    if(intake_ramp && intake_is_holding)
      intakeRamp.config(kIntakeRunDownTime,0, kIntakeHold,0.5);
   
    intake_is_on = false;
    intake_is_holding=false;
    intake_is_off=true;
  }

  public void setIntakeHold() {
    if(intake_ramp && intake_is_on)
      intakeRamp.config(kSpinbackTime,kIntakeSpeed,kIntakeHold,0.5);
    else if(intake_ramp && intake_is_off)
      intakeRamp.config(kIntakeRunDownTime,0,kIntakeHold,0.5);
   
    intake_is_on = false;
    intake_is_off=false;
    intake_is_holding=true;
  }

  public boolean isIntakeOn() {
    return intake_is_on;
  }
  public boolean isIntakeOff() {
    return intake_is_off;
  }
  public boolean isIntakeHolding() {
    return intake_is_holding;
  }

  public boolean isShooterOn() {
    return shooter_is_on;
  }

  public double aveShooterAcc() {
    return shoot.aveAcceleration();
  }

  public double aveShooterVel() {
    return shoot.aveVelocity();
  }

  public double shooterSpeed(){
    return shoot.getRate();
  }
  public double intakeSpeed(){
    return intake.getRate();
  }
  public boolean shooterReady(double target) {
    m_shooter_controller.calculate(aveShooterVel(),target);
    return m_shooter_controller.atSetpoint();
  }

  public boolean intakeReady(double target) {
    m_intake_controller.calculate(intake.aveVelocity(),target);
    return  m_intake_controller.atSetpoint();
  }

  public boolean isBallCaptured() {
    return limit_switch.inContact();
  }

  public void reset() {
    setShooterOff();
   // if(isBallCaptured())
      setIntakeHold();
    //else
    //  setIntakeOff();
  }

  private void setShooterSpeed(){
    if(shooter_ramp)
      shoot.set(shooterRamp.ramp(shoot.getRate()));
    else if(shooter_is_on)
      shoot.set(kShootSpeed);
    else
      shoot.set(kShootHold);
  }
  private void setIntakeSpeed(){
    if(intake_ramp)
      intake.set(intakeRamp.ramp(intake.getRate()));
    else if(intake_is_on)
      intake.set(kIntakeSpeed);
    else
      shoot.set(kIntakeHold);
  }
  @Override
  public void periodic() {
    setShooterSpeed();
    setIntakeSpeed();
   
    //ave_accel = acc_averager.getAve(shoot.getAcceleration());
    //ave_vel = vel_averager.getAve(shoot.getRate());

    // SmartDashboard.putNumber("Intake speed", intake.getRate());
    SmartDashboard.putNumber("Shooter speed", shoot.getRate());
    SmartDashboard.putNumber("Intake speed", intake.getRate());

    kShootSpeed = SmartDashboard.getNumber("Target speed", kShootSpeed);
    SmartDashboard.putBoolean("Ball captured", isBallCaptured());
    SmartDashboard.putBoolean("Intake on", intake_is_on);
    SmartDashboard.putBoolean("Intake hold", intake_is_holding);
    SmartDashboard.putBoolean("Shooter on", shooter_is_on);
  }
  public class Ramp {
    private double m_delay;
    private double m_err;
    private boolean m_ramping=false;
    private double m_start=0;
    private double m_end=0;
    private String m_name;

    private Timer timer = new Timer();
    Ramp(String name){
      m_name=name;
      m_ramping=false;
      timer.start();
    }
    
    public void config(double delay, double start, double end, double err){
      m_delay=delay;
      m_err=err;
      m_start=start;
      m_end=end;
      m_ramping=true;
      System.out.println(m_name+ " config "+start+" "+end+" "+delay);
      timer.reset();
    }
    public void reset(){
      m_ramping=false;
      timer.reset();
    }
    public double ramp(double current){
      if(!m_ramping)
        return 0;
     
      if(Math.abs(current-m_end)<m_err){
        timer.reset();
        return m_end;
      }
      double f = timer.get() / m_delay;
      return MathUtils.lerp(f, 0, 1, m_start, m_end);
      
    }

  }
}
