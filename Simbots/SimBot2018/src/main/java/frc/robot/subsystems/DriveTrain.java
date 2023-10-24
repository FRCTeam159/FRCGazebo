package frc.robot.subsystems;

import gazebo.SimEncMotor;
import gazebo.SimGyro;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithGamepad;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public DriveTrain() {
    super();
  
    leftMotor = new SimEncMotor(RobotMap.FRONTLEFT);
    rightMotor = new SimEncMotor(RobotMap.FRONTRIGHT);
   
    // int ticks_per_foot=(int)(1.0/getFeetPerTick());

    leftMotor.setDistancePerRotation(kDistPerRot);
    rightMotor.setDistancePerRotation(kDistPerRot);

    leftMotor.setInverted();

    //frontRight.configEncoderCodesPerRev((int) ticks_per_foot);
    //backLeft.configEncoderCodesPerRev((int) ticks_per_foot);
    setHighGear();
    gyro.enable();
    log(true);
  }
  
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveWithGamepad());
  }
  public void init() {
		System.out.println("Drivetrain.init");
		enable();
	}
  void log(boolean init) {
    // if (init) {
    //   SmartDashboard.putBoolean("HighGear", true);
    //   SmartDashboard.putNumber("Heading", 0);
    //   SmartDashboard.putNumber("LeftDistance", 0);
    //   SmartDashboard.putNumber("RightDistance", 0);
    //   SmartDashboard.putNumber("Travel", 0);
    //   SmartDashboard.putNumber("LeftWheels", 0);
    //   SmartDashboard.putNumber("RightWheels", 0);
    // } else {
      SmartDashboard.putBoolean("HighGear", !inlowgear);
      SmartDashboard.putNumber("Heading", getHeading());
      SmartDashboard.putNumber("LeftDistance", feet_to_inches(getLeftDistance()));
      SmartDashboard.putNumber("RightDistance", feet_to_inches(getRightDistance()));
      SmartDashboard.putNumber("Travel", feet_to_inches(getDistance()));
      SmartDashboard.putNumber("LeftWheels", round(getLeft()));
      SmartDashboard.putNumber("RightWheels", round(getRight()));
    //}
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

  double round(double x) {
    return 0.001 * Math.round(x * 1000);
  }

  public double getHeading() {
    // if (Robot.isReal())
    return unwrap(last_heading,  gyro.getAngle());
    //return gyro.getAngle();
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
    System.out.println("Drivetrain reset");
    log(true);
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
    log(true);
  }

  public void disable() {
    rightMotor.disable();
    leftMotor.disable();
    log(true);
  }

  public void set(double left, double right) {
    left = coerce(-1.0, 1.0, left);
    right = coerce(-1.0, 1.0, right);
    leftMotor.set(left);
   // leftMotor.set(RobotMap.BACKLEFT);
    rightMotor.set(right);
    //leftMotor.set(RobotMap.FRONTRIGHT);
    log(false);
  }

}
