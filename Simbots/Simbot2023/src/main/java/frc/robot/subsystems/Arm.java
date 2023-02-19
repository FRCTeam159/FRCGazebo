// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gazebo.SimEncMotor;

public class Arm extends Thread {

  public static final int kStageOneChannel = 9;
  public static final int kStageTwoChannel = 10;

    // Solidworks initial offsets to desired pose
  public static final double kStageOneAngleOffset = Math.toRadians(101); // starting angle
  //public static final double kStageOneAngleOffset = Math.toRadians(55.74); // starting angle

  //public static final double kStageTwoAngleOffset = Math.toRadians(161.28);
  public static final double kStageTwoAngleOffset = Math.toRadians(170);

  //public static final double kHoldX = 0.445; // distance corresponding to start angles
  //public static final double kHoldY = 0.115;

  public static double kHoldX = 0; // distance corresponding to start angles
  public static double kHoldY = 0;
  // robot dimensions
  public static final double kwristLength = Units.inchesToMeters(8);

  public static final double kStageOneLength = Units.inchesToMeters(43.18); // 1.0968
  public static final double kStageTwoLength = Units.inchesToMeters(30.59); // 0.7742+
  public static final double kgroundY = Units.inchesToMeters(9.5);   // lower arm joint to ground
  public static final double kfrontX = Units.inchesToMeters(18);     // lower arm joint to front of robot
  public static final double kovertargetY = Units.inchesToMeters(4); // ht of claw over target for grab or place 
  public static final double kgripperX = Units.inchesToMeters(10);    // distance from upper arm to center of claw   
  public static final double kbumperX = Units.inchesToMeters(4);    // width of bumpers   


  // field dimensions

  public static final double[] kcube1 = {0.56,0.36}; // center of platform
  public static final double[] kcube2 = {1.00,0.90};
 
  public static final double[] kcone1 = {0.58,0.87}; // to top of post
  public static final double[] kcone2 = {1.01,1.17};
 
  public static final double[] kshelf =  {0.1,1.0}; // nominal 6" from front of robot (could be zero))
  public static final double[] kground = {0.2,0.2}; // 

  public static boolean debug = false;
 
  private SimEncMotor stageOne;
  private SimEncMotor stageTwo;

  double kMaxAngularSpeed=Math.toRadians(180);
  double kMaxAngularAcceleration=Math.toRadians(90);
  
  boolean test_mode=false;

  // final ProfiledPIDController onePID= 
  // new ProfiledPIDController(4,1,0,
  //  new TrapezoidProfile.Constraints(kMaxAngularSpeed,kMaxAngularAcceleration));
  // final ProfiledPIDController twoPID= 
  // new ProfiledPIDController(4,1,0,
  //   new TrapezoidProfile.Constraints(kMaxAngularSpeed,kMaxAngularAcceleration));
  
  public PIDController onePID = new PIDController(5, 0, 0.0);
  public PIDController twoPID = new PIDController(5, 0, 0.0);

  static double X;
  static double Y;

  static double maxX=1.1;
  static double maxY=1.1;
  static double minX=-1.1;
  static double minY=0.15;

  double[] target =null;
  double oneAngle;
  double twoAngle;
  static int cnt=0;

  public Arm() {
    stageOne = new SimEncMotor(kStageOneChannel);
    stageTwo = new SimEncMotor(kStageTwoChannel);
    stageOne.enable();
    stageTwo.enable();
  }

  public void run() {
    double alpha = lowerArmAngle();
    double beta = upperArmAngle();
    double d[]=getPosition();
    kHoldX=X=d[0];
    kHoldY=Y=d[1];

    System.out.format("initial pose alpha:%-3.1f beta:%-3.1f x:%-1.2f y:%-1.2f\n",
      Math.toDegrees(alpha),Math.toDegrees(beta),X,Y);

    while (!Thread.interrupted()) {
      try {
        Thread.sleep(20);
        setAngle(X, Y);
        setDashboard();
      } catch (Exception ex) {
        System.out.println("exception:" + ex);
      }
    }
  }

  public void setTestMode(){
    test_mode=true;
  }
  public void clrTestMode(){
    test_mode=false;
  }
  public boolean testMode(){
    return test_mode;
  }
  public double getXTarget(){
    return X;
  }
  public double getYTarget(){
    return Y;
  }
  public double getX(){
    return getPosition()[0];
  }
  public double getY(){
   return getPosition()[1];
  }
  public void setX(double x){
    X=x;
    X=X>maxX?maxX:X;
    X=X<minX?minX:X;
  }
  public void setY(double y){
    Y=y;
    Y=Y>maxY?maxY:Y;
    Y=Y<minY?minY:Y;
  }
  public void setPose(double x, double y){
    setX(x);
    setY(y);
  }
  public double getAlpha(){
    return lowerArmAngle();
  }
  public double getBeta(){
    return upperArmAngle();
  }
  public void setAlpha(double a){
    oneAngle=a;
  }
  public void setBeta(double a){
    twoAngle=a;
  }
  public void setTargetPose(double[]p){
    setX(p[0]+kfrontX-kgripperX+kbumperX);
    setY(p[1]-kgroundY+kovertargetY);
  }
  void setDashboard(){
    double d[]=getPosition();
    String s=String.format("X:%-3.1f(%-3.1f) Y:%-3.1f(%-3.1f) A1:%-3.1f(%-3.1f) A2:%-3.1f(%-3.1f)",
      d[0],X,d[1],Y,Math.toDegrees(oneAngle),Math.toDegrees(target[0]),Math.toDegrees(twoAngle),Math.toDegrees(target[1]));
    SmartDashboard.putString("Arm",s);
  }

  // Input x and y, returns 2 angles for the 2 parts of the arm
double[] calculateAngle(double x, double y) {
    double[] angles=new double[2];
   
    double a1 = kStageOneLength;
    double a2 = kStageTwoLength;
    double f1 = (x * x + y * y - a1 * a1 - a2 * a2) / (2 * a1 * a2);

    double q2=-Math.acos(f1);
    if(x<0)
      q2=-q2;
   
    double f2 = a2 * Math.sin(q2) / (a1 + a2 * Math.cos(q2));
    //double q1 = Math.atan2(y,x) - Math.atan(f2);
    double t1=Math.atan2(y,x);
    //double t2=Math.atan2(a2 * Math.sin(q2),a1 + a2 * Math.cos(q2));f2
    double t2=Math.atan(f2);
   
    //double q1 = x<0?t1-t2:t1+t2;// x<0
    double q1 = t1-t2;// x<0

    //System.out.format("q2:%-2.1f t1:%-2.1f t2:%-2.1f q1:%-2.1f\n",
    //Math.toDegrees(q2),Math.toDegrees(t1),Math.toDegrees(t2),Math.toDegrees(q1));

    q2+=q2<0?2*Math.PI:0;

    angles[0] = q1;
    angles[1] = q2;
    
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

  public void setAngle(double x, double y) {
    target = calculateAngle(x, y);
    oneAngle = lowerArmAngle();
    twoAngle = upperArmAngle();
    //if(x<0)
   //   target[1]=-target[1];
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

  public boolean armAtSetPoint() {
    return onePID.atSetpoint() && twoPID.atSetpoint();
  }

  public void setMidCubePose(){
    setTargetPose(kcube1);
  }
  public void setTopCubePose(){
    setTargetPose(kcube2);
  }
  public void setMidConePose(){
    setTargetPose(kcone1);
  }
  public void setTopConePose(){
    setTargetPose(kcone2);
  }
  public void setShelfPose(){
    setTargetPose(kshelf);
  }
  public void setGroundPose(){
    setTargetPose(kground);
  }
 
  public void setHoldingPose() {
    X = kHoldX;
    Y = kHoldY;
  }

}
