// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gazebo.SimEncMotor;

public class Arm extends Thread {

  public static final int kStageOneChannel = 9;
  public static final int kStageTwoChannel = 10;

    // Solidworks initial offsets to desired pose
  public static final double kStageOneAngleOffset = Math.toRadians(55.74); // starting angle
  public static final double kStageTwoAngleOffset = Math.toRadians(161.28);
  public static final double kHoldX = 0.445; // distance corresponding to start angles
  public static final double kHoldY = 0.115;

  // robot dimensions
  public static final double kwristLength = Units.inchesToMeters(8);

  public static final double kStageOneLength = Units.inchesToMeters(43.18); // 1.0968
  public static final double kStageTwoLength = Units.inchesToMeters(30.59)+kwristLength; // 0.7742+
  public static final double kgroundY = Units.inchesToMeters(9.5);   // lower arm joint to ground
  public static final double kfrontX = Units.inchesToMeters(18);     // lower arm joint to front of robot
  public static final double kovertargetY = Units.inchesToMeters(4); // ht of claw over target for grab or place 
  public static final double kgripperX = Units.inchesToMeters(14);    // distance from upper arm to center of claw   

  // field dimensions

  public static final double[] kcube1 = {0.56,0.36}; // center of platform
  public static final double[] kcube2 = {1.00,0.90};
 
  public static final double[] kcone1 = {0.58,0.87}; // to top of post
  public static final double[] kcone2 = {1.01,1.17};
 
  public static final double[] kshelf =  {0.0,1.0}; // nominal 6" from front of robot (could be zero))
  public static final double[] kground = {0.1,0.15}; // 

  public static boolean debug = false;
  public boolean using_Owens_code=false;

  private SimEncMotor stageOne;
  private SimEncMotor stageTwo;

  public PIDController onePID = new PIDController(5, 0, 0);
  public PIDController twoPID = new PIDController(8, 0, 0);

  static double X;
  static double Y;

  double[] target =null;
  double oneAngle;
  double twoAngle;
  static int cnt;

  public Arm() {
    stageOne = new SimEncMotor(kStageOneChannel);
    stageTwo = new SimEncMotor(kStageTwoChannel);
    stageOne.enable();
    stageTwo.enable();
    stageTwo.setInverted();

    onePID.setTolerance(1);
    twoPID.setTolerance(1);
  
  }

  public void run() {
    setHoldingPose();

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

  public void setPose(double[]p){
    X=p[0]+kfrontX-kgripperX;
    Y=p[1]-kgroundY+kovertargetY;
  }
  void setDashboard(){
    double d[]=getPosition();
    String s=String.format("X:%-3.1f(%-3.1f) Y:%-3.1f(%-3.1f) A1:%-3.1f(%-3.1f) A2:%-3.1f(%-3.1f)",
      d[0],d[1],X,Y,Math.toDegrees(oneAngle),Math.toDegrees(target[0]),Math.toDegrees(twoAngle),Math.toDegrees(target[1]));
    SmartDashboard.putString("Arm",s);
  }

  // Input x and y, returns 2 angles for the 2 parts of the arm
  public double[] calculateAngle(double x, double y) {
    double alpha=0;
    double beta=0;

    if(using_Owens_code){
      double[] point = {x, y};
      double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
      alpha = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(distance, 2)-Math.pow(kStageTwoLength, 2))/(2*kStageOneLength*distance))+Math.atan(point[0]/point[1]); // Stage 1 to ground angle
      beta = Math.acos((Math.pow(kStageOneLength, 2)+Math.pow(kStageTwoLength, 2)-Math.pow(distance, 2))/(2*kStageTwoLength*kStageOneLength)); // Top angle
    }
    else{  // from on-line reference
      double a1 = kStageOneLength;
      double a2 = kStageTwoLength;
      double f1 = (x * x + y * y - a1 * a1 - a2 * a2) / (2 * a1 * a2);
      beta = Math.acos(f1);
      double f2 = a2 * Math.sin(beta) / (a1 + a2 * (Math.cos(beta)));
      alpha = Math.atan2(y, x) + Math.atan(f2);
    }
    double[] angles = {alpha, beta};

    return angles;
  }

  public double[] getPosition() {
    double alpha = lowerArmAngle();
    double beta = upperArmAngle();
    return new double[] { kStageOneLength * Math.cos(alpha) + kStageTwoLength * Math.cos(beta - alpha),
        kStageOneLength * Math.sin(alpha) + kStageTwoLength * Math.sin(beta - alpha - Math.PI) };
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
    double oneOut = onePID.calculate(oneAngle, target[0]);
    double twoOut = twoPID.calculate(twoAngle, target[1]);
    if (debug && (cnt % 100) == 0) {
      double pos[] = getPosition();
      System.out.format("angle target:%-3.1f,%-3.1f current:%-3.1f,%-3.1f calc %-3.3f,%-3.3f\n",
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
 
  public void setHoldingPose() {
    X = kHoldX;
    Y = kHoldY;
  }

  public void posTrim(double r) {
    X = X + Math.cos(r);
    Y = Y + Math.sin(r);
  }
}
