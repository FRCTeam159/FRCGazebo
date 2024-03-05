// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import gazebo.SimEncMotor;
import gazebo.SimGyro;
import gazebo.SimMotor;
import gazebo.SimSwitch;
import gazebo.SimContact;

public class Arm extends SubsystemBase implements Constants {

  private SimEncMotor arm = new SimEncMotor(ARM);
  private SimEncMotor shooter = new SimEncMotor(SHOOTER);
  private SimMotor pickup = new SimMotor(PICKUP);
  private SimSwitch armLimit = new SimSwitch(ARM_LIMIT);
  private static SimContact intakeContact = new SimContact(0, 20);
  private static SimContact shootContact = new SimContact(1, 20);
  private static boolean at_starting_position = false;
  private static boolean initialized = false;

  static public double PICKUP_POWER = 0.3;
  static public double PUSH_POWER = 10;
  static public double TARGET_SHOOTER_SPEED = 40;

  public static final double MOVE_RATE = 0.1;

  boolean m_shoot = false;
  boolean m_intake = false;
  boolean m_push = false;
  static boolean incontact = false;
  boolean started = false;

  final SimpleMotorFeedforward m_shooterFF = new SimpleMotorFeedforward(0.01, 0.1);
  final PIDController shooter_pid = new PIDController(0.7, 0, 0);
  
  final SimpleMotorFeedforward m_angleFF = new SimpleMotorFeedforward(0.001, 0.001,0.0001);
  final PIDController arm_pid = new PIDController(0.4, 0.0, 0.0);
  
  public SimGyro m_gyro = new SimGyro(1, SimGyro.Mode.ROLL);

  double armGyroOffset = 149; //imperically measured
  double armEncoderOffset = 0; //imperically measured

  static double starting_angle = 55.0; // imperically determined
  static double target_angle = starting_angle;

  public Arm() {
    arm.reset();
    arm.enable();
    shooter.enable();
    pickup.enable();
    armLimit.enable();
    intakeContact.enable();
    shootContact.enable();
    arm_pid.setTolerance(1, 1);
    shooter_pid.setTolerance(1, 1);
    arm.setDistancePerRotation(360);
    m_gyro.enable();
    m_gyro.reset();
  
  }

  public void setStarted() {
    started = true;
  }

  public void reset() {
    m_shoot = false;
    m_intake = false;
    m_push = false;
    init();
  }

  public static boolean noteAtIntake() {
    return intakeContact.inContact();
  }

  public static boolean noteAtShooter() {
    return shootContact.inContact();
  }

  public static boolean haveNote() {
    return noteAtIntake()||noteAtShooter();
  }

  public void setShooterOn() {
    m_shoot = true;
  }

  public void setShooterOFf() {
    m_shoot = false;
  }

  public void toggleShooter() {
    m_shoot = m_shoot ? false : true;
  }

  public void togglePickup() {
    m_intake = m_intake ? false : true;
  }

  public void setPickupOn() {
    m_intake = true;
  }

  public void setPickupOff() {
    m_intake = false;
  }

  public void togglePusher() {
    m_push = m_push ? false : true;
  }

  public void setPushOn() {
    m_push = true;
  }

  public void setPushOFf() {
    m_push = false;
  }

  public boolean atLowerLimit() {
    return armLimit.lowLimit();
  }

  public boolean atUpperLimit() {
    return armLimit.highLimit();
  }

  public void stepUp(double v) {
    if (!atUpperLimit())
      target_angle += v * MOVE_RATE;
  }

  public void stepDown(double v) {
    if (!atLowerLimit())
      target_angle -= v * MOVE_RATE;
  }

  public void setTargetAngle(double a) {
    if(!Autonomous.running())
      System.out.println("Arm.setTargetAngle "+a);
    target_angle = a;
  }

  public double getAngle() {
    if(initialized)
      return getArmEncoderAngle();
    else
      return getGyroAngle();
  }

  public double getTargetAngle() {
    return target_angle;
  }

  public double getShooterSpeed() {
    return shooter.getRate();
  }

  public boolean atTargetSpeed() {
    return shooter_pid.atSetpoint();
    //return getShooterSpeed() >= TARGET_SHOOTER_SPEED ? true : false;
  }

  public boolean atTargetAngle() {
    return arm_pid.atSetpoint();
  }

  public static boolean atStartingPosition() {
    return at_starting_position;
  }

  public static void init() {
    at_starting_position = false;
    target_angle = starting_angle;
  }

  public boolean isInititialized() {
    return initialized;
  }

  public boolean isStarted() {
    return started;
  }

  @Override
  public void simulationPeriodic(){

  }
  @Override
  public void periodic() {
    if (!Drivetrain.simStarted()) {
      System.out.println("Arm - simulation not started");
      arm.set(0.1); // hold arm up until simulation is started
      return;
    }
    double ffcorr=0;
    double corr=0;
   // initialized = true;
    //angle_pid.setSetpoint(target_angle);
    if(initialized)
      ffcorr=m_angleFF.calculate(getAngle(),arm.getRate());
    else
      ffcorr=m_angleFF.calculate(getAngle());
    corr = arm_pid.calculate(getAngle(),target_angle)+ffcorr;
    arm.set(corr);
    
    if (!at_starting_position) {
      if (atTargetAngle()) {
        at_starting_position = true;
        if (!initialized) {
          arm.reset();
          armEncoderOffset = getGyroAngle();
          initialized = true;
          System.out.println("Initial arm angle=" + armEncoderOffset);
          arm.enable();
        }
        Robot.status = "Arm Ready";
      }
    }
    if (m_shoot){
      ffcorr = m_shooterFF.calculate(TARGET_SHOOTER_SPEED);
      corr=shooter_pid.calculate(shooter.getRate(),TARGET_SHOOTER_SPEED);
      shooter.set(corr+ffcorr);
    }
    else
      shooter.set(-0.1);
    if (m_intake)
      pickup.set(PICKUP_POWER);
    else if (m_push)
      pickup.set(PUSH_POWER);    
    else if(haveNote()){
      if(noteAtShooter())
        pickup.set(-0.3);
      else
        pickup.set(0);
    }
    log();
  }

  public double getGyroAngle() {
    return m_gyro.getRoll() + armGyroOffset;
  }
  public double getArmEncoderAngle() {
    return arm.getDistance()+starting_angle;
  }
  void log() {
    SmartDashboard.putNumber("ArmAngle", getGyroAngle());
    SmartDashboard.putBoolean("Shooting", m_shoot);
    SmartDashboard.putBoolean("Pickup", m_intake);
    SmartDashboard.putBoolean("IntakeSensor", noteAtIntake());
    SmartDashboard.putBoolean("ShootSensor", noteAtShooter());
    SmartDashboard.putNumber("ShootSpeed", shooter.getRate());

  }
}
