// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import gazebo.SimEncMotor;
import gazebo.SimGyro;
import gazebo.SimMotor;
import gazebo.SimSwitch;
import gazebo.SimContact;

public class Arm extends SubsystemBase implements Constants {

  private SimMotor motor = new SimMotor(ARM);

  private SimEncMotor shooter = new SimEncMotor(SHOOTER);
  private SimMotor pickup = new SimMotor(PICKUP);
  private SimSwitch armLimit = new SimSwitch(ARM_LIMIT);
  private static SimContact intakeContact = new SimContact(0, 10);
  private static SimContact shootContact = new SimContact(1, 5);
  private static boolean at_starting_position = false;
  private static boolean initialized = false;

  static public double SHOOT_POWER = 8;
  static public double PICKUP_POWER = 1;
  static public double PUSH_POWER = 1;
  static public double TARGET_SHOOTER_SPEED = 80;

  public static final double MOVE_RATE = 0.1;

  boolean shooter_on = false;
  boolean pickup_on = false;
  boolean pusher_on = false;
  public static String status;
  static boolean incontact = false;
  boolean started = false;

  final PIDController pid = new PIDController(0.1, 0.00, 0.00);

  public SimGyro m_gyro = new SimGyro(1, SimGyro.Mode.ROLL);

  double armGyroOffset = 149;//

  static double starting_angle = 55.0;
  static double target_angle = starting_angle;

  public Arm() {
    motor.enable();
    shooter.enable();
    pickup.enable();
    armLimit.enable();
    intakeContact.enable();
    shootContact.enable();
    pid.setTolerance(1, 1);
    m_gyro.enable();
    m_gyro.reset();
  }

  public void setStarted() {
    started = true;

  }

  public void reset() {
    shooter_on = false;
    pickup_on = false;
    pusher_on = false;
    init();
  }

  public static boolean noteAtIntake() {
    return intakeContact.inContact();
  }

  public static boolean noteAtShooter() {
    return shootContact.inContact();
  }

  public void setShooterOn() {
    shooter_on = true;
  }

  public void setShooterOFf() {
    shooter_on = false;
  }

  public void toggleShooter() {
    shooter_on = shooter_on ? false : true;
  }

  public void togglePickup() {
    pickup_on = pickup_on ? false : true;
  }

  public void setPickupOn() {
    pickup_on = true;
  }

  public void setPickupOff() {
    pickup_on = false;
  }

  public void togglePusher() {
    pusher_on = pusher_on ? false : true;
  }

  public void setPusherOn() {
    pusher_on = true;
  }

  public void setPusherOFf() {
    pusher_on = false;
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
    target_angle = a;
  }

  public double getAngle() {
    return getGyroAngle();
  }

  public double getTargetAngle() {
    return target_angle;
  }

  public double getShooterSpeed() {
    return shooter.getRate();
  }

  public boolean atTargetSpeed() {
    return getShooterSpeed() >= TARGET_SHOOTER_SPEED ? true : false;
  }

  public boolean atTargetAngle() {
    return pid.atSetpoint();
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
  public void periodic() {
    if (!Drivetrain.simStarted()) {
      motor.set(0.5);
      return;
    }

    initialized = true;

    double angle = getAngle();
    pid.setSetpoint(target_angle);
    double corr = pid.calculate(angle);
    motor.set(corr);

    if (!at_starting_position) {
      if (atTargetAngle()) {
        at_starting_position = true;
        status = "Ready";
      }
    }
    if (shooter_on)
      shooter.set(SHOOT_POWER);
    else
      shooter.set(-0.1);
    if (pickup_on || pusher_on)
      pickup.set(PICKUP_POWER);
    else
      pickup.set(-0.03);

    log();
  }

  public double getGyroAngle() {
    if (initialized)
      return m_gyro.getRoll() + armGyroOffset;
    else
      return starting_angle;
  }

  void log() {
    SmartDashboard.putNumber("Arm", getAngle());
    SmartDashboard.putBoolean("Shooting", shooter_on);
    SmartDashboard.putBoolean("Pickup", pickup_on);
    SmartDashboard.putBoolean("Sensor1", noteAtIntake());
    SmartDashboard.putBoolean("Sensor2", noteAtShooter());
    SmartDashboard.putNumber("Shooter", shooter.getRate());
    SmartDashboard.putString("Status", status);

  }
}
