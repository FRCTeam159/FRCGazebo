// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import gazebo.SimEncMotor;

public class Arm extends Thread {

  public static final int kStageOneChannel = 9;
  public static final int kStageTwoChannel = 10;

  public static final double kStageOneLength = Units.inchesToMeters(43.18); // 1.0968
  public static final double kStageTwoLength = Units.inchesToMeters(30.59); // 0.7742
  public static final double kStageOneAngleOffset=Math.toRadians(55.74); // starting angle
  public static final double kStageTwoAngleOffset=Math.toRadians(161.28); 
  public static final double kHoldX=0.445; // distance corresponding to start angles
  public static final double kHoldY=0.115; 
  public static final double kgroundY=0.115; 

  public static boolean trace=false;

  private SimEncMotor stageOne;
	private SimEncMotor stageTwo;

  public PIDController onePID = new PIDController(5, 0, 0);
  public PIDController twoPID = new PIDController(3, 0, 0);

  static double X;
  static double Y;
  static int cnt;
  
  public Arm() {
    stageOne=new SimEncMotor(kStageOneChannel);
    stageTwo=new SimEncMotor(kStageTwoChannel);
    stageOne.enable();
    stageTwo.enable();

    onePID.setTolerance(1);
    twoPID.setTolerance(1);
  }

  public void run() {
    setHoldingPose();

    while (!Thread.interrupted()) {
      try {
        Thread.sleep(50);
        setAngle(X,Y);
      } catch (Exception ex) {
        System.out.println("exception:" + ex);
      }
    }
  }
  // Input x and y, returns 2 angles for the 2 parts of the arm
  public double[] calculateAngle(double x, double y) {
    double a1=kStageOneLength;
    double a2=kStageTwoLength;
    double f1=(x*x+y*y-a1*a1-a2*a2)/(2*a1*a2);
    double q2=Math.acos(f1);
    double f2=a2*Math.sin(q2)/(a1+a2*(Math.cos(q2)));
    double q1=Math.atan2(y,x)+Math.atan(f2);
   
    double[] angles = {q1, q2};

    return angles;
  }

  public double[] getPosition(){
    double alpha = lowerArmAngle();
    double beta = upperArmAngle();
      return new double[] {kStageOneLength*Math.cos(alpha) + kStageTwoLength*Math.cos(beta-alpha), 
      kStageOneLength*Math.sin(alpha) + kStageTwoLength*Math.sin(beta-alpha-Math.PI)};
  }

   public double lowerArmAngle(){
      return stageOne.getDistance()*2*Math.PI+kStageOneAngleOffset;
   }
   public double upperArmAngle(){
    return stageTwo.getDistance()*2*Math.PI+kStageTwoAngleOffset;
 }
  public void setAngle(double x, double y) {
    double[] target = calculateAngle(x, y);
    double oneAngle=lowerArmAngle();
    double twoAngle=upperArmAngle();
    double oneOut = onePID.calculate(oneAngle, target[0]);
    double twoOut = twoPID.calculate(twoAngle, target[1]);
    if(trace && (cnt%100)==0){
      double pos[]= getPosition();
      System.out.format("angle target:%-3.1f,%-3.1f current:%-3.1f,%-3.1f calc %-3.3f,%-3.3f\n",
        Math.toDegrees(target[0]),Math.toDegrees(target[1]),
        Math.toDegrees(oneAngle),Math.toDegrees(twoAngle),
        pos[0],pos[1]);
    }
     stageOne.set(oneOut);
     stageTwo.set(twoOut);
     cnt++;
  }

  public boolean armAtSetPoint(){
    return onePID.atSetpoint() && twoPID.atSetpoint();
  }

  public void setHoldingPose(){
    X=kHoldX;
    Y=kHoldY;
  }
  public void posTrim(double r){
    X = X+Math.cos(r);
    Y = Y+Math.sin(r);
  }
}
