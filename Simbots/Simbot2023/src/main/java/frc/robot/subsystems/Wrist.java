package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gazebo.SimEncMotor;
import gazebo.SimPiston;
import static frc.robot.Constants.*;

public class Wrist extends Thread {

  double kMaxAngularSpeed=Math.toRadians(360);
  double kMaxAngularAcceleration=Math.toRadians(180);

  final ProfiledPIDController rotatePID= 
  new ProfiledPIDController(3,1,0,
    new TrapezoidProfile.Constraints(kMaxAngularSpeed,kMaxAngularAcceleration));
  final ProfiledPIDController twistPID= 
  new ProfiledPIDController(0.8,0,0,
  new TrapezoidProfile.Constraints(kMaxAngularSpeed,kMaxAngularAcceleration));
  // public PIDController rotatePID = new PIDController(4, 1, 0);
  // public PIDController twistPID = new PIDController(4, 0.5, 0);


  private SimEncMotor twistMotor;
  private SimEncMotor rotateMotor;

  static double twistAngle = 0;
  static double rotateAngle = 0;

  SimPiston m_piston = new SimPiston(0);

  static int cnt = 0;

  static boolean debug = false;

  boolean m_twistup=false;
  boolean m_clawopen=false;

  public static final double kInit = Math.toRadians(0);
  public static final double kDrive = Math.toRadians(0);
  
  public static final double kGround = Math.toRadians(0);
  public static final double kShelf = Math.toRadians(49);

  public static final double kConeMiddle = Math.toRadians(48);
  public static final double kConeTop = Math.toRadians(91);

  public static final double kCubeMiddle = Math.toRadians(45);
  public static final double kCubeTop = Math.toRadians(119);
  public Wrist() {

    rotateMotor = new SimEncMotor(kWristRotateChannel);
    twistMotor = new SimEncMotor(kWristTwistChannel);
    rotateMotor.enable();
    twistMotor.enable();
    twistMotor.setInverted();
    m_piston.enable();

    rotatePID.setTolerance(1);
    twistPID.setTolerance(1);
  }

  public void run() {
    setInitialPose();
    closeClaw();

    while (!Thread.interrupted()) {
      try {
        Thread.sleep(20);
        setRotation();
        setTwist();
        setDashboard();    
        cnt++;
      } catch (Exception ex) {
        System.out.println("exception:" + ex);
      }
    }
  }

  public void setDrivePose(){
    twistAngle=0;
    rotateAngle=kGround;
  }
  public void setGroundPose(){
    twistAngle=0;
    rotateAngle=kGround;
  }
  public void setMidConePose(){
    twistAngle=0;
    rotateAngle=kConeMiddle;
  }
  public void setTopConePose(){
    twistAngle=0;
    rotateAngle=kConeTop;
  }
  public void setShelfPose(){
    twistAngle=0;
    rotateAngle=kShelf;
  }
  public void setMidCubePose(){
    twistAngle=0;
    rotateAngle=kCubeMiddle;
  }
  public void setTopCubePose(){
    twistAngle=0;
    rotateAngle=kCubeTop;
  }
 
  void setDashboard(){
    String s=String.format("Rot:%-3.1f(%-3.1f) Twist:%-3.1f(%-3.1f) Claw:%s",
      Math.toDegrees(getRotation()),Math.toDegrees(rotateAngle),Math.toDegrees(getTwist()),Math.toDegrees(twistAngle),m_clawopen?"Open":"Closed");
    SmartDashboard.putString("Wrist",s);
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

  public double getTwist() {
    return twistMotor.getDistance() * 2 * Math.PI + kTwistAngleOffset;
  }

  public double getRotation() {
    return rotateMotor.getDistance() * 2 * Math.PI + kRotateAngleOffset;
  }

  public void setTwist(double r){
    twistAngle=r;
  }
  public void setRotation(double r){
    rotateAngle=r;
  }
  public void setInitialPose() {
    rotateAngle = kInit; 
    twistAngle = -kTwistAngleOffset;
  }

  public void setHoldingPose() {
    rotateAngle = kInit-kRotateAngleOffset; // rotate to level
    twistAngle = -kTwistAngleOffset;
  }
  public boolean twistUp(){
    return m_twistup;
  }
  public void twist(){
    m_twistup=true;
    twistAngle=Math.toRadians(180)-kTwistAngleOffset;
    System.out.println("wrist twist:"+twistAngle);

  }
  public void untwist(){
    m_twistup=false;
    twistAngle=Math.toRadians(0)-kTwistAngleOffset;
    System.out.println("wrist untwist:"+twistAngle);
  } 

  public boolean clawOpen(){
    return m_clawopen;
  }
  public void openClaw() {
    m_clawopen=true;
    System.out.println("wrist openclaw");
    m_piston.forward();
  }

  public void closeClaw() {
    m_clawopen=false;
    System.out.println("wrist closeclaw");
    m_piston.reverse();
  }
}
