package frc.robot.subsystems;

import gazebo.SimEncMotor;
import gazebo.SimGyro;
import utils.Averager;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithGamepad;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;


/**
 *
 */
public class DriveTrain extends SubsystemBase{

  private SimEncMotor leftMotor;
  private SimEncMotor rightMotor;
  
  private SimGyro gyro = new SimGyro(0);
 
 // private DoubleSolenoid gearPneumatic = new DoubleSolenoid(0, 1);

  static double WHEEL_DIAMETER = 6.1; // inches
  //static int ENCODER_TICKS = Robot.isReal() ? 1024 : 360;
  //static int ticks_per_foot=(int)(1.0/Math.PI*WHEEL_DIAMETER/12/ENCODER_TICKS);
  //static double ticks_per_foot = ENCODER_TICKS * 12.0 / Math.PI / WHEEL_DIAMETER; // ~1146 ticks/foot
  static final double feetpermeter=3.28084;
  public static final double kDistPerRot = feetpermeter*Units.inchesToMeters(WHEEL_DIAMETER * Math.PI);
	private double last_heading=0;

  boolean inlowgear = false;
  static double f2m=0.3048;

  private final DifferentialDriveOdometry odometry;

  private Averager acc_averager = new Averager(20);
  private Averager vel_averager = new Averager(5);
  private double ave_acc;
  private double ave_vel;
  private double last_velocity=0;
  private double last_time=0;
  private Timer timer=new Timer();

  public DriveTrain() {
    super();
  
    leftMotor = new SimEncMotor(RobotMap.FRONTLEFT);
    rightMotor = new SimEncMotor(RobotMap.FRONTRIGHT);
   
    // int ticks_per_foot=(int)(1.0/getFeetPerTick());

    leftMotor.setDistancePerRotation(kDistPerRot);
    rightMotor.setDistancePerRotation(kDistPerRot);

    leftMotor.setInverted();

    odometry = new DifferentialDriveOdometry(getRotation2d(), 0, 0);
    setHighGear();
    gyro.enable();
    timer.start();
    log();
  }
  
  public Rotation2d getRotation2d(){
		double angle;
		angle=getHeading();
		return Rotation2d.fromDegrees(angle);
	}
  /** Updates the field-relative position. */
	public void updateOdometry() {
		double l=f2m*leftMotor.getDistance();
		double r=f2m*rightMotor.getDistance();
		odometry.update(getRotation2d(), l, r);
	}

  public Pose2d getPose() {
    return odometry.getPoseMeters();
}
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveWithGamepad());
  }
  public void init() {
		System.out.println("Drivetrain.init");
		enable();
    timer.reset();
	}
  void log() {
      SmartDashboard.putBoolean("HighGear", !inlowgear);
      SmartDashboard.putNumber("Heading", getHeading());
      SmartDashboard.putNumber("LeftDistance", feet_to_inches(getLeftDistance()));
      SmartDashboard.putNumber("RightDistance", feet_to_inches(getRightDistance()));
      SmartDashboard.putNumber("Travel", feet_to_inches(getDistance()));
      SmartDashboard.putNumber("LeftWheels", round(getLeft()));
      SmartDashboard.putNumber("RightWheels", round(getRight()));
      SmartDashboard.putNumber("Velocity", Math.abs(getAveVelocity()));
      SmartDashboard.putNumber("Acceleration", Math.abs(getAveAcceleration()));
  }

  @Override
	public void periodic() {
		updateOdometry();
   
		log();
	}

  public double getAcceleration() {
    double tm=timer.get();
    double vel=getVelocity();
    double accel=0;
    if(last_time>0 && tm>0 && tm>last_time){
        accel=(vel-last_velocity)/(tm-last_time);
    }
    last_velocity=vel;
    last_time=tm; 
    return accel; 
}
  public double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }
  public static double feet_to_inches(double x) {
    return Math.round(12 * x * 100.0 / 100);
  }

  public double getRight() {
    return rightMotor.get();
  }

  public double getLeft() {
    return leftMotor.get();
  }

  public double getRightDistance() {
    return rightMotor.getDistance();
  }

  public double getLeftDistance() {
    return leftMotor.getDistance();
  }

  public double getDistance() {
    double d1 = getRightDistance();
    double d2 = getLeftDistance();
    return 0.5 * (d1 + d2);
  }

  public double getLeftVelocity() {
    return leftMotor.getRate();
  }

  public double getRightVelocity() {
    return rightMotor.getRate();
  }

  public double getVelocity() {
    double d1 = getLeftVelocity();
    double d2 = getRightVelocity();
    return 0.5 * (d1 + d2);
  }

  public double getAveVelocity(){
    return ave_vel;
  }
  public double getAveAcceleration(){
    return ave_acc;

  }
  double round(double x) {
    return 0.001 * Math.round(x * 1000);
  }

  public double getHeading() {
    return unwrap(last_heading,  gyro.getAngle());
  }

  public boolean inLowGear() {
    return inlowgear;
  }

  public void setLowGear() {
    if (!inlowgear) {
      //gearPneumatic.set(DoubleSolenoid.Value.kReverse);
      System.out.println("Setting Low Gear");
      inlowgear = true;
    }
  }

  public void setHighGear() {
    if (inlowgear) {
      //gearPneumatic.set(DoubleSolenoid.Value.kForward);
      System.out.println("Setting High Gear");
      inlowgear = false;
    }
  }

  double coerce(double min, double max, double x) {
    if (x < min)
      x = min;
    else if (x > max)
      x = max;
    return x;
  }

  public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {

    // local variables to hold the computed PWM values for the motors
    double leftMotorOutput;
    double rightMotorOutput;
    if(inlowgear)
      moveValue*=0.7;
    if (squaredInputs) {
      // square the inputs (while preserving the sign) to increase fine control
      // while permitting full power
      if (moveValue >= 0.0) {
        moveValue = (moveValue * moveValue);
      } else {
        moveValue = -(moveValue * moveValue);
      }
      if (rotateValue >= 0.0) {
        rotateValue = (rotateValue * rotateValue);
      } else {
        rotateValue = -(rotateValue * rotateValue);
      }
    }

    if (moveValue > 0.0) {
      if (rotateValue > 0.0) {
        leftMotorOutput = moveValue - rotateValue;
        rightMotorOutput = Math.max(moveValue, rotateValue);
      } else {
        leftMotorOutput = Math.max(moveValue, -rotateValue);
        rightMotorOutput = moveValue + rotateValue;
      }
    } else {
      if (rotateValue > 0.0) {
        leftMotorOutput = -Math.max(-moveValue, rotateValue);
        rightMotorOutput = moveValue + rotateValue;
      } else {
        leftMotorOutput = moveValue - rotateValue;
        rightMotorOutput = -Math.max(-moveValue, -rotateValue);
      }
    }
    set(leftMotorOutput, rightMotorOutput);
    //System.out.println("Left="+leftMotorOutput+" right="+rightMotorOutput);
  }

  /**
   * Reset the robots sensors to the zero states.
   */
  public void reset() {
    resetEncoders();
    resetGyro();
    last_heading=0;
    Pose2d p=new Pose2d(0,0,new Rotation2d(0));
    odometry.resetPosition(getRotation2d(),0,0,p);
    last_velocity=0;
    last_time=0;
    timer.reset();
    System.out.println("Drivetrain reset");
    log();
  }

  public void resetEncoders() {
    rightMotor.reset();
    leftMotor.reset();
  }

  public void resetGyro() {
    System.out.println("Gyro reset");
    gyro.reset();
  }

  public void enable() {
    rightMotor.enable();
    leftMotor.enable();
    log();
  }

  public void disable() {
    rightMotor.disable();
    leftMotor.disable();
    log();
  }

  public void set(double left, double right) {
    leftMotor.set(left);
    rightMotor.set(right);
    ave_acc = acc_averager.getAve(getAcceleration());
    ave_vel = vel_averager.getAve(getVelocity());
    log();
  }

}
