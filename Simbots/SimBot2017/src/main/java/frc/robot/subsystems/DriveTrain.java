package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithGamepad;

import gazebo.SimEncMotor;
import gazebo.SimGyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase implements RobotMap {

	private SimEncMotor leftMotor;
	private SimEncMotor rightMotor;

	private SimGyro gyro = new SimGyro(0);

	// private DoubleSolenoid gearPneumatic = new DoubleSolenoid(0,1);

	static double WHEEL_DIAMETER = 3; // inches
	public static final double f2m=0.3048; // feet to meters
	static final double i2m=0.0254; // inches to meters

	public static final double kTrackWidth = i2m*2*23; // bug? need to double actual value for geometry to work
	public static final double kWheelDiameter = i2m*8; // wheel diameter in tank model

    public static final double kDistPerRot = Units.inchesToMeters(WHEEL_DIAMETER * Math.PI);

	boolean inlowgear = false;

	public static double kMaxVelocity = 1.0; // meters per second
	public static double kMaxAcceleration = 0.5; //  meters/second/second
	public static double kMaxAngularSpeed = 720; // degrees per second

	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(2*kTrackWidth);
    final DifferentialDriveOdometry odometry;

	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1,1);
	private final PIDController leftPIDController = new PIDController(0.5, 0.0, 0.0);
	private final PIDController rightPIDController = new PIDController(0.5, 0.0, 0.0);

	private final SlewRateLimiter speedLimiter = new SlewRateLimiter(kMaxVelocity);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(kMaxAngularSpeed);

	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithGamepad());
	}

	public DriveTrain() {
		super();
		leftMotor = new SimEncMotor(LEFTWHEELS);
		rightMotor = new SimEncMotor(RIGHTWHEELS);
		leftMotor.setDistancePerRotation(kDistPerRot);
    	rightMotor.setDistancePerRotation(kDistPerRot);
		setLowGear();
		gyro.enable();
		log();
		reset();
		odometry = new DifferentialDriveOdometry(getRotation2d(), 0, 0);
	}
	
	public Rotation2d getRotation2d(){
		double angle;
		angle=getHeading();
		return Rotation2d.fromDegrees(angle);
	}
  /** Updates the field-relative position. */
	public void updateOdometry() {
		double l=leftMotor.getDistance();
		double r=rightMotor.getDistance();
		odometry.update(getRotation2d(), l, r);
	}
	public void resetOdometry(Pose2d p) {
		odometry.resetPosition(getRotation2d(),0,0,p);
	}
  	public Pose2d getPose() {
    	return odometry.getPoseMeters();
  	}

  	@Override
	public void periodic() {
		updateOdometry();
		log();
	}

	public void enable() {
		//gyro.reset();
		rightMotor.enable();
		leftMotor.enable();
		log();
	}

	public void disable() {
		rightMotor.disable();
		leftMotor.disable();
		log();
	}

	void log() {	
		SmartDashboard.putBoolean("HighGear", !inlowgear);
		SmartDashboard.putNumber("Heading", getHeading());
		SmartDashboard.putNumber("LeftDistance", getLeftDistance());
		SmartDashboard.putNumber("RightDistance", getRightDistance());
		SmartDashboard.putNumber("Travel", getDistance());
		SmartDashboard.putNumber("LeftWheels", round(leftMotor.get()));
		SmartDashboard.putNumber("RightWheels", round(rightMotor.get()));
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
		double x = 0.5 * (d1 + d2);
		return x;
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
		return gyro.getAngle();
	}

	public void setLowGear() {
		if (!inlowgear) {
			System.out.println("Setting Low Gear");
			inlowgear = true;
		}
	}

	public void setHighGear() {
		if (inlowgear) {
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

	public void tankDrive(double left, double right) {
		leftMotor.set(left);
		rightMotor.set(right);
		log();
	}

	public void set(double left, double right) {
		tankDrive(left, right);
	}

	public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
		final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
		final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
		final double leftOutput = leftPIDController.calculate(leftMotor.getRate(), speeds.leftMetersPerSecond);
		final double rightOutput = rightPIDController.calculate(rightMotor.getRate(),speeds.rightMetersPerSecond);
		set(leftOutput + leftFeedforward,rightOutput + rightFeedforward);
	}

	public void drive(double xSpeed, double rot) {
	    xSpeed = speedLimiter.calculate(xSpeed);
		rot = rotLimiter.calculate(rot);
		var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
		setSpeeds(wheelSpeeds);
	}
	// A teleop drive method that uses odometry and kinetemtics
    public void odometryDrive(double xSpeed, double rot) {
		drive(xSpeed, rot);
		log();
	}
	public void arcadeDrive(double moveValue, double rotateValue) {
		// local variables to hold the computed PWM values for the motors
		double leftMotorOutput;
		double rightMotorOutput;

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
		set(leftMotorOutput,rightMotorOutput);	
	}
	public void resetGyro(){
		gyro.enable();
		gyro.reset();
	}
	/**
	 * Reset the robots sensors to the zero states.
	 */
	public void reset() {
		rightMotor.reset();
		leftMotor.reset();
	}
}
