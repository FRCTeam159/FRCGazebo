// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Gyro;
import frc.robot.objects.SparkMotor;
import utils.MathUtils;

public class DriveTrain extends SubsystemBase {

	private SparkMotor leftMotor;
	private SparkMotor rightMotor;

	public Gyro gyro = new Gyro(0);

	public static final double kTrackWidth = i2M(2*23); // bug? need to double actual value for geometry to work
	public static final double kWheelDiameter = i2M(7.8); // wheel diameter in tank model
	
	public static double kMaxVelocity = 1.36; // meters per second
	public static double kMaxAcceleration = 0.25; //  meters/second/second
	public static double kMaxAngularSpeed = 180; // degrees per second

	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(2*kTrackWidth);
	private final DifferentialDriveOdometry odometry;

	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1,1);
	private final PIDController leftPIDController = new PIDController(0.1, 0.0, 0.0);
	private final PIDController rightPIDController = new PIDController(0.1, 0.0, 0.0);

	private final SlewRateLimiter speedLimiter = new SlewRateLimiter(kMaxVelocity);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(kMaxAngularSpeed);

	private double distancePerRotation=kWheelDiameter*Math.PI;

	public static final int FRONT_LEFT = 1;
    public static final int FRONT_RIGHT = 2;
	
	public boolean enable_gyro = true;
	private double last_heading=0;
	public boolean arcade_mode=false;
	public boolean dampen_tilt=false;

	private Pose2d field_pose;


	public DriveTrain() {
		leftMotor = new SparkMotor(FRONT_LEFT);
		rightMotor = new SparkMotor(FRONT_RIGHT);
		leftMotor.setDistancePerRotation(distancePerRotation);
		rightMotor.setDistancePerRotation(distancePerRotation);
		leftMotor.setScale(1);
		rightMotor.setScale(1);
		//leftMotor.setMaxAccel(kMaxAcceleration);
		//rightMotor.setMaxAccel(kMaxAcceleration);
	
		rightMotor.setInverted();
	
		odometry = new DifferentialDriveOdometry(getRotation2d(), distancePerRotation, distancePerRotation);
		//odometry = new DifferentialDriveOdometry(0,0,0);
		
		SmartDashboard.putBoolean("Enable gyro", enable_gyro);
		SmartDashboard.putBoolean("Arcade mode", arcade_mode);
	}
	private static double i2M(double inches) {
		return inches * 0.0254;
	}
	private double r2D(double radians) {
		return  180*radians/Math.PI;
	}
	private static double coerce(double min, double max, double value) {
		return Math.max(min, Math.min(value, max));
	}
	
	public void init(){
		System.out.println("Drivetrain.init");
		field_pose=getPose();
		enable();
	}
	public void disable(){
		System.out.println("Drivetrain.disable");
		leftMotor.disable();
		rightMotor.disable();
		gyro.disable();
	}
	public void enable(){
		System.out.println("Drivetrain.enable");
		leftMotor.enable();
		rightMotor.enable();
		gyro.enable();
	}
	// get transform from pose
	public static Transform2d getTransform(Pose2d pose){
		return new Transform2d(pose.getTranslation(), pose.getRotation());
	}
	// add two poses
	public static Pose2d add(Pose2d p1,Pose2d p2){
		Transform2d td=getTransform(p2);
		return p1.plus(td);
	}
	public void reset(){
		field_pose=add(field_pose,getPose());
		System.out.println("Drivetrain.reset");
		leftMotor.reset();
		rightMotor.reset();
		gyro.reset();
		last_heading =0;
	}
	public void resetPose() {
		resetOdometry(new Pose2d(0,0,new Rotation2d(0)));
		field_pose=getPose();
    }
	public double getHeading(){
		return getRotation2d().getDegrees();
	}
	public double gyroHeading(){
		return gyro.getHeading();
	}
	public double calcHeading(){
		double angle=r2D((getRightDistance()-getLeftDistance())/kTrackWidth);	
		return angle;
	}
	public Rotation2d getRotation2d(){
		double angle;
		if(enable_gyro)
			angle=gyroHeading();
		else
			angle=calcHeading();

		angle=unwrap(last_heading,angle);
		last_heading=angle;
		return Rotation2d.fromDegrees(angle);
	}
	public double getLeftDistance(){
		return  leftMotor.getDistance();
	}
	public double getRightDistance(){
		return  rightMotor.getDistance();
	}
	public double getLeftVelocity(){
		return  leftMotor.getRate();
	}
	public double getRightVelocity(){
		return  rightMotor.getRate();
	}
	public double getVelocity(){
		return  0.5*(getLeftVelocity()+getRightVelocity());
	}
	public double getDistance(){
		return  0.5*(getLeftDistance()+getRightDistance());
	}
	public double aveVelocity(){
		return  0.5*(leftMotor.aveVelocity()+rightMotor.aveVelocity());
	}
	public double aveAcceleration(){
		return  0.5*(leftMotor.aveAcceleration()+rightMotor.aveAcceleration());
	}
	public void log(){
		SmartDashboard.putNumber("Heading", getHeading());
		SmartDashboard.putNumber("Distance", getDistance());
		SmartDashboard.putNumber("Velocity", aveVelocity());
		SmartDashboard.putNumber("Accel", aveAcceleration());
		enable_gyro=SmartDashboard.getBoolean("Enable gyro", enable_gyro); 
		arcade_mode=SmartDashboard.getBoolean("Arcade mode", arcade_mode);	
	}
	@Override
	public void periodic() {
		updateOdometry();
		log();
	}

	@Override
	public void simulationPeriodic() {    
		updateOdometry();
		log();
	}
	
	public void drive(double xSpeed, double rot) {
	    xSpeed = speedLimiter.calculate(xSpeed);
		rot = rotLimiter.calculate(rot);
		var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
		setSpeeds(wheelSpeeds);
	}
	public void tankDrive(double xSpeed, double rot) {
		Translation2d speeds=arcadeToTank(xSpeed,rot);
		DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(speeds.getX(),speeds.getY());
		setSpeeds(wheelSpeeds);
	}

	public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
		final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
		final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

		final double leftOutput = leftPIDController.calculate(leftMotor.getRate(), speeds.leftMetersPerSecond);
		final double rightOutput = rightPIDController.calculate(rightMotor.getRate(),speeds.rightMetersPerSecond);
		set(leftOutput + leftFeedforward,rightOutput + rightFeedforward);
	}

    /** Updates the field-relative position. */
	public void updateOdometry() {
		double l=leftMotor.getDistance();
		double r=rightMotor.getDistance();
		odometry.update(getRotation2d(), l, r);
	}
	public void resetOdometry(Pose2d pose) {
		leftMotor.reset();
		rightMotor.reset();
		last_heading = 0;
		gyro.reset();
		//odometry.resetPosition(pose, getRotation2d());
		odometry.resetPosition(getRotation2d(),0,0,pose );

	}
	public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
	public Pose2d getFieldPose() {
        return field_pose;
    }

	public void set(double left,double right) {
		double damping=1.0;
		if(enable_gyro && dampen_tilt){
			double lift=Math.abs(gyro.getRoll());
			damping=MathUtils.lerp(lift, 0.5, 4, 1, 0.1);
		}
		leftMotor.set(left*damping);
		rightMotor.set(right*damping);
		log();
	}
	public void set(double value) {
		leftMotor.set(value);
		rightMotor.set(value);
		log();
	}
	// A teleop drive method that uses odometry and kinetemtics
    public void odometryDrive(double xSpeed, double rot) {
		drive(xSpeed, rot);
		log();
	}
	// removes heading discontinuity at 180 degrees
    public static double unwrap(double previous_angle, double new_angle) {
        double d = new_angle - previous_angle;
        d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
        return previous_angle + d;
    }
	public Translation2d arcadeToTank(double moveValue, double turnValue){
		double leftMotorOutput;
		double rightMotorOutput;
		if (moveValue > 0.0) {
			if (turnValue > 0.0) {
				leftMotorOutput = Math.max(moveValue, turnValue);
				rightMotorOutput = moveValue - turnValue;
			} else {
				leftMotorOutput = moveValue + turnValue;
				rightMotorOutput = Math.max(moveValue, -turnValue);
			}
		} else {
			if (turnValue > 0.0) {
				leftMotorOutput = moveValue + turnValue;
				rightMotorOutput = -Math.max(-moveValue, turnValue);
			} else {
				leftMotorOutput = -Math.max(-moveValue, -turnValue);
				rightMotorOutput = moveValue - turnValue;
			}
		}
		return new Translation2d(leftMotorOutput,rightMotorOutput);
	}
	// A simple teleop drive method that sets motor voltages only
	public void arcadeDrive(double moveValue, double turnValue) {
		double leftMotorOutput;
		double rightMotorOutput;
		
		Translation2d speeds=arcadeToTank(moveValue,turnValue);
		// Make sure values are between -1 and 1
		leftMotorOutput = coerce(-1, 1, speeds.getX());
		rightMotorOutput = coerce(-1, 1, speeds.getY());
		set(leftMotorOutput,rightMotorOutput);
	}
}
