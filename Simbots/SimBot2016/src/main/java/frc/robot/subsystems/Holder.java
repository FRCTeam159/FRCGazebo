// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.ExecHolder;
import gazebo.SimEncMotor;
import gazebo.SimMotor;
import gazebo.SimRangefinder;
import gazebo.SimSwitch;

public class Holder extends SubsystemBase implements RobotMap {

  static final double GATE_OPEN_SPEED = 0.25;
  static final double GATE_REMOVE_SPEED = -0.5;
  static final double GATE_CLOSE_SPEED = -0.3;
  static final double GATE_OFF_SPEED = 0.0;

  static final double PUSH_SPEED = 1.0;
  static final double PUSH_HOLD_SPEED = 0.02;
  static final double PUSH_REMOVE_SPEED = -0.25;
  static final double PUSH_OFF_SPEED = 0.0;

  static final double FORWARDLIMITPOSITION = (80.0 / 360.0);
  static final double REVERSELIMITPOSITION = (10.0 / 360.0);

  static final double I = 0.0005;
  static final double D = 0.1;

  static final double BALL_DETECT_VALUE = 0.5;

  boolean initialized = false;
  double push_hold_speed;
  boolean pushRequested = false;
  double gateTicksPerRevolution;
  SimEncMotor gateMotor=new SimEncMotor(SHOOTER_GATE_MOTOR);
  SimMotor pushMotor=new SimMotor(PUSHWHEEL_MOTOR);
  SimRangefinder ballSensor = new SimRangefinder(SHOOTER_RANGEFINDER);

  SimSwitch gateSwitch = new SimSwitch(SHOOTER_GATE_SWITCH);

  boolean found_zero = false;

  /** Creates a new Holder. */
  public Holder() {
    gateMotor.enable();
    pushMotor.enable();
    ballSensor.enable();
    gateSwitch.enable();
  }

  public void initDefaultCommand() {
    setDefaultCommand(new ExecHolder());
  }

  public void log() {
    //SmartDashboard.putBoolean("Holder Initialized", isInitialized());
    SmartDashboard.putBoolean("Gate Open", isGateOpen());
    SmartDashboard.putBoolean("Gate Closed", isGateClosed());
    SmartDashboard.putBoolean("Ball present", isBallPresent());
  }

  void init() {
    initialized = false;
    pushMotor.set(PUSH_OFF_SPEED);
    gateMotor.set(GATE_OFF_SPEED);
  }

  public boolean isGateOpen() {
    return gateSwitch.highLimit();
  }

  public boolean isGateClosed() {
    return gateSwitch.lowLimit();
  }

  public boolean isBallPresent() {
    double distance = ballSensor.getDistance();
    return distance < BALL_DETECT_VALUE ? true : false;
  }

  public void openGate() {
    if (!isGateOpen())
      gateMotor.set(GATE_OPEN_SPEED);
    else
      gateMotor.set(GATE_OFF_SPEED);
  }

  public void closeGate() {
    if (!isGateClosed())
      gateMotor.set(GATE_CLOSE_SPEED);
    else
      gateMotor.set(GATE_OFF_SPEED);
  }

  public void pushBall() {
    pushMotor.set(PUSH_SPEED);
    pushRequested = true;
  }

  public void holdBall() {
    pushRequested = false;
    gateMotor.set(GATE_OPEN_SPEED);
    pushMotor.set(push_hold_speed);
  }

  public void removeBall() {
    gateMotor.set(GATE_REMOVE_SPEED);
    pushMotor.set(PUSH_REMOVE_SPEED);
  }

  public boolean isInitialized() {
    return initialized;
  }

  void setInitialized() {
    initialized = true;
    gateMotor.set(GATE_OFF_SPEED);
  }

  public void initialize() {
    if (!isGateClosed())
      gateMotor.set(GATE_CLOSE_SPEED);
    else
      gateMotor.set(GATE_OFF_SPEED);
  }

  public void reset() {
    // initialized=false;
    gateMotor.set(GATE_OFF_SPEED);
    pushMotor.set(PUSH_OFF_SPEED);
    gateMotor.set(GATE_CLOSE_SPEED);
  }

  public void setPushHoldSpeed(double d) {
    push_hold_speed = d;
  }

  public boolean pushRequested() { 
    return pushRequested;
  }
  public boolean findZero() {
    if (found_zero)
      return true;
    else {
      gateMotor.set(GATE_CLOSE_SPEED);
      if (gateSwitch.lowLimit()) {
        found_zero = true;
        // gateMotor.ConfigLimitMode(CANTalon::kLimitMode_SoftPositionLimits);
        gateMotor.reset();
        // gateMotor.ConfigSoftPositionLimits(FORWARDLIMITPOSITION,REVERSELIMITPOSITION);
        return true;
      }
    }
    return false;
  }

}
