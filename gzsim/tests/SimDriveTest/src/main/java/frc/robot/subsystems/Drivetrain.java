// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import gazebo.SimEncMotor;

import gazebo.SimGyro;

public class Drivetrain extends SubsystemBase implements Constants {
	private SimEncMotor leftMotor;
	private SimEncMotor rightMotor;

	private Simulation simulation;
	
	private SimGyro gyro = new SimGyro();

	public static final double kTrackWidth = i2M(20); // inches
	public static final double kWheelDiameter = i2M(8); // wheel radius in tank model
	public static final double kMaxVelocity = 2;
    public static final double kMaxAcceleration = 1.5;

	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
	private final DifferentialDriveOdometry odometry;

	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.5,1);
	private final PIDController leftPIDController = new PIDController(0.5, 0, 0.0);
	private final PIDController rightPIDController = new PIDController(0.5, 0, 0.0);

	private final Field2d m_fieldSim = new Field2d();

	// For Gazebo simulation use "Calibrate" auto to determine where 
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

		leftMotor = new SimEncMotor(FRONT_LEFT);
		rightMotor = new SimEncMotor(FRONT_RIGHT);
		leftMotor.setDistancePerRotation(r2M(1.0));
		rightMotor.setDistancePerRotation(r2M(1.0));

		leftMotor.setScale(scale);
		rightMotor.setScale(scale);
	
		rightMotor.setInverted();

		odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
		SmartDashboard.putData("Field", m_fieldSim);
        //simulation.init();
		//enable();
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
	}

	private static double coerce(double min, double max, double value) {
		return Math.max(min, Math.min(value, max));
	}
	public double getTime(){
		return simulation.getSimTime();
	}
	public void startAuto(){
		simulation.reset();
		simulation.start();
		enable();
	}
	public void init(){
		System.out.println("Drivetrain.init");
		//simulation.init();
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
		//simulation.run();
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
	}
	public double getHeading(){
		return gyro.getHeading();
	}
	public double getLeftDistance(){
		return  leftMotor.getDistance() ;
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
		//SmartDashboard.putNumber("Power Scale",scale);
	}
	@Override
	public void periodic() {
		updateOdometry();
        m_fieldSim.setRobotPose(odometry.getPoseMeters());
	}

	@Override
	public void simulationPeriodic() {
		updateOdometry();
		log();
	}
	
	public void drive(double xSpeed, double rot) {
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
		//System.out.println(l+" "+r);
		odometry.update(
			gyro.getRotation2d(), l, r);
	}
	public void resetOdometry(Pose2d pose) {
		leftMotor.reset();
		rightMotor.reset();
		gyro.reset();
		odometry.resetPosition(pose, gyro.getRotation2d());
	}

    public Pose2d getPose() {
		//updateOdometry();
        return odometry.getPoseMeters();
    }
	public void set(double value) {
		leftMotor.set(value);
		rightMotor.set(value);
		log();
	}
    public void odometryDrive(double xSpeed, double rot) {
		drive(xSpeed, rot);
		log();
	}
}
