// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.objects.ArmPosition;
import gazebo.SimEncMotor;
import static frc.robot.Constants.*;

public class Arm extends Thread {
  
  public static double kHoldX = 0; // distance corresponding to start angles
  public static double kHoldY = 0;

  public static double kmaxAngleError=Math.toRadians(1.0);

  public static final double[] kinit =  {0.131,0.378,0}; // from kStageOneAngleOffset, kStageTwoAngleOffset

  public static final double[] kcube1 = {0.6,0.6,Math.toRadians(45)}; // center of platform
  public static final double[] kcube2 = {1.00,1.1,Math.toRadians(119)};
 
  public static final double[] kcone1 = {0.64,0.84,Math.toRadians(59)}; // to top of post
  public static final double[] kcone2 = {1.05,1.17,Math.toRadians(98)};
 
  public static final double[] kshelf =  {0.1,1.0,Math.toRadians(49)}; // nominal 6" from front of robot (could be zero))
  public static final double[] kground = {0.3,0.2, Math.toRadians(0)}; // 

  public static boolean debug = false;
 
  private SimEncMotor stageOne;
  private SimEncMotor stageTwo;
  private SimEncMotor twistMotor;
  private SimEncMotor rotateMotor;

  final ProfiledPIDController onePID= 
  new ProfiledPIDController(8,1,0,
   new TrapezoidProfile.Constraints(kMaxLowerArmAngularSpeed,kMaxLowerArmAngularAcceleration));
  final ProfiledPIDController twoPID= 
  new ProfiledPIDController(10,1,0,
    new TrapezoidProfile.Constraints(kMaxUpperArmAngularSpeed,kMaxUpperArmAngularAcceleration));
  final ProfiledPIDController rotatePID= 
  new ProfiledPIDController(3,1,0,
    new TrapezoidProfile.Constraints(kMaxWristRotateAngularSpeed,kMaxWristRotaterAcceleration));
  final ProfiledPIDController twistPID= 
  new ProfiledPIDController(0.8,0,0,
    new TrapezoidProfile.Constraints(kMaxWristRotateAngularSpeed,kMaxWristRotaterAcceleration));
 
  static double X;
  static double Y;
  static double twistAngle = 0;
  static double rotateAngle = 0;

  static double maxX=1.1;
  static double maxY=1.2;
  static double minX=-1.2;
  static double minY=0.15;

  double[] target =null;
  double oneAngle;
  double twoAngle;
  static int cnt=0;

  ArmPosition m_armpose;

  public Arm() {
    stageOne = new SimEncMotor(kStageOneChannel);
    stageTwo = new SimEncMotor(kStageTwoChannel);
    rotateMotor = new SimEncMotor(kWristRotateChannel);
    twistMotor = new SimEncMotor(kWristTwistChannel);
    stageOne.enable();
    stageTwo.enable();
    rotateMotor.enable();
    twistMotor.enable();
    twistMotor.setInverted();
    rotatePID.setTolerance(1);
    twistPID.setTolerance(1);
  }

  public void run() {
    //double alpha = lowerArmAngle();
    //double beta = upperArmAngle();

    setPose(kinit);
    // double d[]=getPosition();
    
    // kinit[0]=X=d[0];
    // kinit[1]=Y=d[1];
    
    //System.out.format("initial pose alpha:%-3.1f beta:%-3.1f x:%-1.3f y:%-1.3f\n",
    //  Math.toDegrees(alpha),Math.toDegrees(beta),X,Y);

    while (!Thread.interrupted()) {
      try {
        Thread.sleep(20);
        setAngles();
        setRotation();
        setTwist();
        log();
      } catch (Exception ex) {
        System.out.println("exception:" + ex);
      }
    }
  }

  public void setTwist(double r){
    twistAngle=r;
  }
  public void setRotation(double r){
    rotateAngle=r;
  }
  public double getTwist() {
    return twistMotor.getDistance() * 2 * Math.PI;
  }

  public double getRotation() {
    return rotateMotor.getDistance() * 2 * Math.PI + kRotateAngleOffset;
  }

  private void setTwist() {
    double angle = getTwist();
    double err = twistPID.calculate(angle, twistAngle);
    if (debug && cnt % 20 == 0) {
      System.out.format("twist target:%-2.1f current:%-2.1f corr:%-1.3f\n", twistAngle, angle, err);
    }
    twistMotor.set(err);
  }

  private void setRotation() {
    double angle = getRotation();
    double err = rotatePID.calculate(angle, rotateAngle);
    if (debug && cnt % 20 == 0) {
      System.out.format("rotate target:%-2.1f current:%-2.1f corr:%-1.3f\n", rotateAngle, angle, err);
    }
    rotateMotor.set(err);
  }
  public double[] getTargetPosition(){
    double tmp[]={X,Y};
    return tmp;
  }
  public double getXTarget(){
    return X;
  }
  public double getYTarget(){
    return Y;
  }
  
  void setX(double x){
    X=x;
    X=X>maxX?maxX:X;
    X=X<minX?minX:X;
  }
  void setY(double y){
    Y=y;
    Y=Y>maxY?maxY:Y;
    Y=Y<minY?minY:Y;
  }
  public void setPose(double x, double y){
    double tmp[]={x,y,rotateAngle};
    setPose(tmp);    
  }
 
  void setPose(double[]p){
    setX(p[0]);
    setY(p[1]);
    rotateAngle=p[2];
    twistAngle=0;
   // m_armpose=new ArmPosition(p[0], p[1],ArmPosition.consType.pose); // unused so far
  }
  
  void log(){
    double d[]=getPosition();
    String s=String.format("X:%-3.2f(%-3.2f) Y:%-3.1f(%-3.2f) A1:%-3.1f(%-3.1f) A2:%-3.1f(%-3.1f)",
      d[0],X,
      d[1],Y,
      Math.toDegrees(oneAngle),Math.toDegrees(target[0]),
      Math.toDegrees(twoAngle),Math.toDegrees(target[1]));
    SmartDashboard.putString("Arm",s);
    s=String.format("Rot:%-3.1f(%-3.1f) Twist:%-3.1f(%-3.1f)",
    Math.toDegrees(getRotation()),Math.toDegrees(rotateAngle),Math.toDegrees(getTwist()),Math.toDegrees(twistAngle));
    SmartDashboard.putString("Wrist",s);
  }

  // Input x and y, returns 2 angles for the 2 parts of the arm
double[] calculateAngle(double x, double y) {  
    double a1 = kStageOneLength;
    double a2 = kStageTwoLength;
    double f1 = (x * x + y * y - a1 * a1 - a2 * a2) / (2 * a1 * a2);

    double beta=-Math.acos(f1);
    if(x<0)
      beta=-beta;
   
    double f2 = a2 * Math.sin(beta) / (a1 + a2 * Math.cos(beta));
    double t1=Math.atan2(y,x);
    double t2=Math.atan(f2);
    double alpha = t1-t2;
    beta+=beta<0?2*Math.PI:0;

    double[] angles={alpha,beta}; 
    return angles;
  }

  public double[] getPosition() {
    double alpha = lowerArmAngle();
    double beta = upperArmAngle();
    double a1 = kStageOneLength;
    double a2 = kStageTwoLength;
    
    double x=a1 * Math.cos(alpha) + a2 * Math.cos(alpha+beta);
    double y=a1 * Math.sin(alpha) + a2 * Math.sin(alpha+beta);
    return new double[] {x ,y};
  }

  public double lowerArmAngle() {
    return stageOne.getDistance() * 2 * Math.PI + kStageOneAngleOffset;
  }

  public double upperArmAngle() {
    return stageTwo.getDistance() * 2 * Math.PI + kStageTwoAngleOffset;
  }

  void setAngles() {
    target = calculateAngle(X, Y);
    oneAngle = lowerArmAngle();
    twoAngle = upperArmAngle();
   
    double oneOut = onePID.calculate(oneAngle, target[0]);
    double twoOut = twoPID.calculate(twoAngle, target[1]);
    if (debug && (cnt % 100) == 0) {
      double pos[] = getPosition();
      System.out.format("X:%-1.2f Y:%-1.2f angle target:%-3.1f,%-3.1f current:%-3.1f,%-3.1f calc %-3.3f,%-3.3f\n",
          X,Y,
          Math.toDegrees(target[0]), Math.toDegrees(target[1]),
          Math.toDegrees(oneAngle), Math.toDegrees(twoAngle),
          pos[0], pos[1]);
    }
    stageOne.set(oneOut);
    stageTwo.set(twoOut);
    cnt++;
  }

  public boolean atSetPoint() {
    double err1=oneAngle-target[0];
    double err2=twoAngle-target[1];
    if(Math.abs(err1)<kmaxAngleError && Math.abs(err2)<kmaxAngleError){
      return true;
    }
    return false;
  }

  public void setMidCubePose(){
    setPose(kcube1);
  }
  public void setTopCubePose(){
    setPose(kcube2);
  }
  public void setMidConePose(){
    setPose(kcone1);
  }
  public void setTopConePose(){
    setPose(kcone2);
  }
  public void setShelfPose(){
    setPose(kshelf);
  }
  public void setGroundPose(){
    setPose(kground);
  }
  public void setInitPose() {
    setPose(kinit);
  }
}
