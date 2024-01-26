// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import gazebo.SimEncMotor;
import gazebo.SimMotor;
import gazebo.SimSwitch;

public class Arm extends SubsystemBase implements Constants{

  private SimEncMotor motor=new SimEncMotor(ARM);

  private SimEncMotor shooter=new SimEncMotor(SHOOTER);
  private SimMotor pickup=new SimMotor(PICKUP);
  private SimMotor pusher=new SimMotor(PUSHER);
  SimSwitch armLimit = new SimSwitch(ARM_LIMIT);
  private boolean initialized=false;
  static int cnt=0;
  static public double SHOOT_POWER=10;
  static public double PICKUP_POWER=1;
  static public double PUSH_POWER=1;

  public static final double MOVE_RATE=0.1;

  boolean shooter_on=false;
  boolean pickup_on=false; 
  boolean pusher_on=false;

  final PIDController pid=new PIDController(0.1,0.00,0.00);

  double target_angle=0.0;
  public Arm() {
    setTargetAngle(0.0);
    motor.setDistancePerRotation(360);
  
    motor.enable();
    shooter.enable();
    pickup.enable();
    pusher.enable();
    armLimit.enable();
  }

  public void reset(){
    shooter_on=false;
    pickup_on=false;
    pusher_on=false;
    target_angle=SPEAKER_SHOOT_ANGLE;

  }

  public void toggleShooter(){
    shooter_on=shooter_on?false:true;
  }

  public void togglePickup(){
    pickup_on=pickup_on?false:true;
  }

  public void togglePusher(){
    pusher_on=pusher_on?false:true;
  }
  public boolean atLowerLimit() {
    return armLimit.lowLimit();
  }
  public boolean atUpperLimit() {
    return armLimit.highLimit();
  }
 
  public void stepUp(double v){
    if(!atUpperLimit())
    target_angle += v * MOVE_RATE;
  }
  public void stepDown(double v){
    if(!atLowerLimit())
      target_angle -= v * MOVE_RATE;
  }
  public void setTargetAngle(double a){
      target_angle=a;
  }
  public double getAngle(){
    return motor.getDistance();
  }
  public double getTargetAngle(){
    return target_angle;
  }
  @Override
  public void periodic() {
     double angle = getAngle();

    if (!initialized) {
       motor.set(-1);
      if (atLowerLimit()) {
        System.out.println("Arm low limit reached");
        initialized = true;
        motor.reset();
        target_angle = SPEAKER_SHOOT_ANGLE;
        pid.setSetpoint(target_angle);
      }
    }
    else{
      pid.setSetpoint(target_angle);
      double corr = pid.calculate(angle);
      //double err=angle-target_angle;
      //if (cnt % 100 == 0) 
      //  System.out.println("target:"+target_angle+" angle:" + angle +" err:"+err+" corr:" + corr);
      
      motor.set(corr);
      if(shooter_on)
        shooter.set(SHOOT_POWER);
      else
        shooter.set(0);
      if(pickup_on)
        pickup.set(PICKUP_POWER);
      else
        pickup.set(-0.05);
      if(pusher_on)
        pusher.set(PUSH_POWER);
      else
        pusher.set(-0.0);
    }
    cnt++;
    SmartDashboard.putNumber("Arm", angle);
    SmartDashboard.putBoolean("Shooter", shooter_on);
    SmartDashboard.putBoolean("Pickup", pickup_on);
    SmartDashboard.putBoolean("Pusher", pusher_on);
    SmartDashboard.putNumber("ShooterSpeed", shooter.getRate());
  }
}
