// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import gazebo.SimCamera;
import gazebo.SimEncMotor;

import gazebo.SimGyro;

public class Drivetrain extends SubsystemBase {

	private SimCamera camera;
	private SimEncMotor leftMotor;
	private SimEncMotor rightMotor;

	private Simulation simulation;
	
	private SimGyro gyro = new SimGyro();

	public static final double kTrackWidth = i2M(2*23); // bug? need to double actual value for geometry to work
	public static final double kWheelDiameter = i2M(7.8); // wheel diameter in tank model
	public static final double kMaxVelocity = 1;
    public static final double kMaxAcceleration = 1;
	public static double kMaxSpeed = i2M(100); // inches per second
	public static double kMaxAngularSpeed = 180; // degrees per second

	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
	private final DifferentialDriveOdometry odometry;

	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.25,0.5);
	private final PIDController leftPIDController = new PIDController(0.25, 0, 0.0);
	private final PIDController rightPIDController = new PIDController(0.25, 0, 0.0);

	private final SlewRateLimiter speedLimiter = new SlewRateLimiter(kMaxSpeed);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(kMaxAngularSpeed);

	private double distancePerRotation=kWheelDiameter*Math.PI;

	public static final int FRONT_LEFT = 1;
    public static final int FRONT_RIGHT = 2;

	public static final int FRONT_CAMERA = 1;
	
	private final Field2d m_fieldSim = new Field2d();
	public boolean enable_gyro = false;
	public boolean reversed=false;
	private double last_heading=0;

	
	// For Gazebo simulation use "Calibrate" auto routine to determine where 
	// model velocity stops increasing with input power
	// 1) set scale to 1 and in Calibrate set max power to ~5 step size to 1 etc.
	// 2) run calibrate (record max power at max velocity)
	// 3) set scale to max_velocity (full power range should now be -1 to 1)
	// 4) re-run Calibrate to verify (set max power to 1 step to 0.1 etc.)
	// note: may need to reduce PID and feed-forward values above as well to avoid jitters
	private double scale=2.5; // 

	/** Creates a new Subsystem. */
	public Drivetrain() {
		simulation = new Simulation(this);

		camera=new SimCamera(FRONT_CAMERA);
		simulation.addCamera(camera);

		leftMotor = new SimEncMotor(FRONT_LEFT);
		rightMotor = new SimEncMotor(FRONT_RIGHT);
		leftMotor.setDistancePerRotation(distancePerRotation);
		rightMotor.setDistancePerRotation(distancePerRotation);

		leftMotor.setScale(scale);
		rightMotor.setScale(scale);
	
		rightMotor.setInverted();
		
		odometry = new DifferentialDriveOdometry(getRotation2d());
		SmartDashboard.putData("Field", m_fieldSim);
		SmartDashboard.putBoolean("record", false);
		SmartDashboard.putBoolean("Enable gyro", enable_gyro); 
	}

	private static double i2M(double inches) {
		return inches * 0.0254;
	}
	private static double m2I(double meters) {
		return meters / 0.0254;
	}
	private double r2M(double rotations) {
		return  Math.PI * kWheelDiameter*rotations;
	}
	private double r2D(double radians) {
		return  180*radians/Math.PI;
	}
	private static double coerce(double min, double max, double value) {
		return Math.max(min, Math.min(value, max));
	}
	public double getTime(){
		return simulation.getClockTime();
	}
	public void startAuto(){
		simulation.reset();
		simulation.start();
		enable();
	}
	public void init(){
		System.out.println("Drivetrain.init");
		simulation.init();
		enable();
	}
	public void disable(){
		System.out.println("Drivetrain.disable");
		leftMotor.disable();
		rightMotor.disable();
		gyro.disable();
		simulation.end();
	}
	public void enable(){
		simulation.run();
		System.out.println("Drivetrain.enable");
		leftMotor.enable();
		rightMotor.enable();
		gyro.enable();
	}
	public void reset(){
		//simulation.reset();
		System.out.println("Drivetrain.reset");
		leftMotor.reset();
		rightMotor.reset();
		gyro.reset();
		last_heading = 0;
		reversed=false;
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
		if(reversed)
			angle+=180;
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
	public void log(){
		SmartDashboard.putNumber("Heading", getHeading());
		SmartDashboard.putNumber("Left distance", getLeftDistance());
		SmartDashboard.putNumber("Right distance", getRightDistance());
		SmartDashboard.putNumber("Left speed", getLeftVelocity());
		SmartDashboard.putNumber("Right speed", getRightVelocity());
		enable_gyro=SmartDashboard.getBoolean("Enable gyro", enable_gyro); 	
	}
	@Override
	public void periodic() {
		updateOdometry();
        m_fieldSim.setRobotPose(odometry.getPoseMeters());
	}

	@Override
	public void simulationPeriodic() {
		updateOdometry();
		// capture a movie from the Gazebo camera (gztest/c++/tmp/test_0.avi)
		if(SmartDashboard.getBoolean("record", false))
			simulation.startCamera(FRONT_CAMERA);
		else
			simulation.stopCamera(FRONT_CAMERA);
		log();
	}
	
	public void drive(double xSpeed, double rot) {
	    xSpeed = speedLimiter.calculate(xSpeed);
		rot = rotLimiter.calculate(rot);
		var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
		setSpeeds(wheelSpeeds);
	}

	public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
		final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
		final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

		final double leftOutput = leftPIDController.calculate(leftMotor.getRate(), speeds.leftMetersPerSecond);
		final double rightOutput = rightPIDController.calculate(rightMotor.getRate(),speeds.rightMetersPerSecond);
		leftMotor.set(leftOutput + leftFeedforward);
		rightMotor.set(rightOutput + rightFeedforward);
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
		odometry.resetPosition(pose, getRotation2d());
	}

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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
	// A simple teleop drive method that sets motor voltages only
	public void arcadeDrive(double moveValue, double turnValue) {
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
		// Make sure values are between -1 and 1
		leftMotorOutput = coerce(-1, 1, leftMotorOutput);
		rightMotorOutput = coerce(-1, 1, rightMotorOutput);
		leftMotor.set(leftMotorOutput);
		rightMotor.set(rightMotorOutput);
		log();
	}
}
