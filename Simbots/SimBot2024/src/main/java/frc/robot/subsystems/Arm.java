// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import gazebo.SimEncMotor;
import gazebo.SimSwitch;

public class Arm extends SubsystemBase implements Constants{

  private SimEncMotor motor=new SimEncMotor(ARM);
  SimSwitch armLimit = new SimSwitch(ARM_LIMIT);
  private boolean initialized=false;
  static int cnt=0;

  public static final double MOVE_RATE=0.1;

  final PIDController pid=new PIDController(0.005,0.00,0.00);

  double target_angle=0.0;
  public Arm() {
    setTargetAngle(0.0);
    motor.setDistancePerRotation(360);
    //motor.setInverted();
    motor.enable();
    armLimit.enable();
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
      double err=angle-target_angle;
      if (cnt % 100 == 0) 
        System.out.println("target:"+target_angle+" angle:" + angle +" err:"+err+" corr:" + corr);
      
      motor.set(corr);
    cnt++;
    SmartDashboard.putNumber("Arm", angle);
  }
    
  }
}
