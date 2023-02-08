package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import gazebo.SimEncMotor;

public class Wrist extends Thread {
  public PIDController rotatePID = new PIDController(2, 0, 0);
  public PIDController twistPID = new PIDController(2, 0, 0);

  public static final int kWristRotateChannel = 11;
  public static final int kWristTwistChannel = 12;

  private SimEncMotor twistMotor;
  private SimEncMotor rotateMotor;

  public static final double kRotateAngleOffset = Math.toRadians(25); // starting angle
  public static final double kTwistAngleOffset = Math.toRadians(4); // starting angle

  static double twistAngle = 0; // rotate to level
  static double rotateAngle = 0;

  static int cnt=0;

  public Wrist() {

    rotateMotor = new SimEncMotor(kWristRotateChannel);
    twistMotor = new SimEncMotor(kWristTwistChannel);
    rotateMotor.enable();
    twistMotor.enable();
    twistMotor.setInverted();
    //rotateMotor.setInverted();

    rotatePID.setTolerance(1);
    twistPID.setTolerance(1);
  }

  public void run() {
    setHoldingPose();

    while (!Thread.interrupted()) {
      try {
        Thread.sleep(50);
        setRotation();
        setTwist();
        cnt++;

      } catch (Exception ex) {
        System.out.println("exception:" + ex);
      }
    }
  }

  private void setTwist() {
    double angle=getTwist();
    double err = twistPID.calculate(angle, twistAngle);
    if(cnt%20==0){
      System.out.format("twist target:%-2.1f current:%-2.1f corr:%-1.3f\n",twistAngle,angle,err);
    }
    twistMotor.set(err);
  }

  private void setRotation() {
    double angle=getRotation();
    double err = rotatePID.calculate(angle, rotateAngle);
    if(cnt%20==0){
      System.out.format("rotate target:%-2.1f current:%-2.1f corr:%-1.3f\n",rotateAngle,angle,err);
    }
    rotateMotor.set(err);
  }

  private double getTwist() {
    return twistMotor.getDistance()*2*Math.PI+kTwistAngleOffset;
  }

  private double getRotation() {
    return rotateMotor.getDistance()*2*Math.PI+kRotateAngleOffset;

  }

  public void setHoldingPose() {
    rotateAngle = -kRotateAngleOffset; // rotate to level
    twistAngle = -kTwistAngleOffset;
  }

}
