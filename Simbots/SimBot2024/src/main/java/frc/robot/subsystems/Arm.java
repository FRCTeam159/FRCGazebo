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
import gazebo.SimContact;

public class Arm extends SubsystemBase implements Constants{

  private SimEncMotor motor=new SimEncMotor(ARM);

  private SimEncMotor shooter=new SimEncMotor(SHOOTER);
  private SimMotor pickup=new SimMotor(PICKUP);
  private SimMotor pusher=new SimMotor(PUSHER);
  private SimSwitch armLimit = new SimSwitch(ARM_LIMIT);
  private SimContact noteContact= new SimContact(0,5);
  private static boolean at_starting_position=false;
  private static boolean initialized=false;

  static public double SHOOT_POWER=7;
  static public double PICKUP_POWER=0.5;
  static public double PUSH_POWER=1;
  static public double TARGET_SHOOTER_SPEED=90;

  public static final double MOVE_RATE=0.1;

  boolean shooter_on=false;
  boolean pickup_on=false; 
  boolean pusher_on=false;
  public static String status;
  boolean incontact=false;
  boolean started=false;

  final PIDController pid=new PIDController(0.2,0.00,0.00);

  double target_angle=0.0;
  public Arm() {
    setTargetAngle(0.0);
    motor.setDistancePerRotation(360);
  
    motor.enable();
    shooter.enable();
    pickup.enable();
    pusher.enable();
    armLimit.enable();
    noteContact.enable();
    target_angle=getAngle();
    pid.setTolerance(1,1);
  }

  public void setStarted(){
    started=true;

  }
  public void reset(){
    shooter_on=false;
    pickup_on=false;
    pusher_on=false;
    init();
    //target_angle=SPEAKER_SHOOT_ANGLE;
  }

   public void checkContact() {
    boolean b=noteContact.inContact();
    boolean n=noteContact.newState();
    if(b && n)
      incontact=true;
    if(n && !b)
      incontact=false;
  
  }
  public boolean isNoteCaptured() {
    checkContact();
    return incontact;
  }
  public void setShooterOn(){
    shooter_on=true;
  }
  public void setShooterOFf(){
    shooter_on=false;
  }

  public void toggleShooter(){
    shooter_on=shooter_on?false:true;
  }

  public void togglePickup(){
    pickup_on=pickup_on?false:true;
  }

  public void setPickupOn(){
    pickup_on=true;
  }
  public void setPickupOff(){
    pickup_on=false;
  }
  public void togglePusher(){
    pusher_on=pusher_on?false:true;
  }

  public void setPusherOn(){
    pusher_on=true;
  }
  public void setPusherOFf(){
    pusher_on=false;
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
  public double getShooterSpeed(){
    return shooter.getRate();
  }
  public boolean atTargetSpeed(){
    return  getShooterSpeed()>=TARGET_SHOOTER_SPEED?true:false;
  }
  public boolean atTargetAngle(){
    return pid.atSetpoint();
  }
  public static boolean atStartingPosition(){
    return at_starting_position;

  }
  public static void init(){
    initialized=false;
    at_starting_position=false;
  }

  public boolean isInititialized(){
    return initialized;

  }
  public boolean isStarted(){
    return started;

  }
  public boolean findZero(){
    motor.set(-1);
    if (atLowerLimit()) {
          System.out.println("Arm low limit reached");
          initialized = true;
          motor.reset();
          target_angle = SPEAKER_SHOOT_ANGLE;
          pid.setSetpoint(target_angle);
          status = "Initializing .. ";
          return true;
     }
     return false;
  }
  @Override
  public void periodic() {
      double angle = getAngle();
      pid.setSetpoint(target_angle);
      double corr = pid.calculate(angle);
      motor.set(corr);
      if(!started){
        log();
        status = "NotStarted";
        return;
      }

      if (!at_starting_position) {
        if (atTargetAngle()) {
          at_starting_position = true;
          status = "Ready";
        }
      }
      if (shooter_on)
        shooter.set(SHOOT_POWER);
      else
        shooter.set(0);
      if (pickup_on)
        pickup.set(PICKUP_POWER);
      else
        pickup.set(0.03);
      if (pusher_on)
        pusher.set(PUSH_POWER);
      else
        pusher.set(-0.02);
    
    log();
  }

  void log(){
    SmartDashboard.putNumber("Arm", getAngle());
    SmartDashboard.putBoolean("Shooting", shooter_on);
    SmartDashboard.putBoolean("Pickup", pickup_on);
    SmartDashboard.putBoolean("Pushing", pusher_on);
    SmartDashboard.putBoolean("Note", isNoteCaptured());
    SmartDashboard.putNumber("Shooter", shooter.getRate());
    SmartDashboard.putString("Status", status);

  }
}
