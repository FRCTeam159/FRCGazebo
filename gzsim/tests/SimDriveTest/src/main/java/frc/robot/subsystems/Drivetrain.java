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
import gazebo.SimMotor;
import gazebo.SimEncoder;
import gazebo.SimGyro;

public class Drivetrain extends SubsystemBase implements Constants {
	private SimMotor leftMotor;
	private SimMotor rightMotor;
	private SimEncoder leftEncoder;
	private SimEncoder rightEncoder;
	private SimGyro gyro = new SimGyro();

	public static final double kTrackWidth = i2M(28.25); // inches
	private static final double kWheelRadius = i2M(3); // 8 inch wheel diameter in tank model

	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
	private final DifferentialDriveOdometry odometry;

	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 0.5);
	private final PIDController leftPIDController = new PIDController(0.25, 0, 0);
	private final PIDController rightPIDController = new PIDController(0.25, 0, 0);

	private final Field2d m_fieldSim = new Field2d();

	/** Creates a new Subsystem. */
	public Drivetrain() {
		leftMotor = new SimMotor(FRONT_LEFT);
		rightMotor = new SimMotor(FRONT_RIGHT);
		leftEncoder=new SimEncoder(FRONT_LEFT);
		rightEncoder=new SimEncoder(FRONT_RIGHT);
		rightEncoder.setInverted();

		odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
		SmartDashboard.putData("Field", m_fieldSim);

		enable();
	}

	private static double i2M(double inches) {
		return inches * 0.0254;
	}
	private static double m2I(double meters) {
		return meters / 0.0254;
	}
	private double r2M(double rotations) {
		return rotations * 2 * Math.PI * kWheelRadius;
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

	public void disable(){
		System.out.println("Drivetrain.disable");
		leftMotor.disable();
		rightMotor.disable();
		leftEncoder.disable();
		rightEncoder.disable();
		gyro.disable();
	}
	public void enable(){
		System.out.println("Drivetrain.enable");
		leftMotor.enable();
		rightMotor.enable();
		leftEncoder.enable();
		rightEncoder.enable();
		gyro.enable();
	}
	public void reset(){
		System.out.println("Drivetrain.reset");
		leftEncoder.reset();
		rightEncoder.reset();
		gyro.reset();
	}
	public double getHeading(){
		return gyro.getHeading();
	}
	public double getLeftDistance(){
		return  r2M(leftEncoder.getDistance()) ;
	}
	public double getRightDistance(){
		return  r2M(rightEncoder.getDistance());
	}
	public double getLeftVelocity(){
		return  r2M(leftEncoder.getRate());
	}
	public double getRightVelocity(){
		return  r2M(rightEncoder.getRate());
	}
	public void log(){
		SmartDashboard.putNumber("Heading", getHeading());
		SmartDashboard.putNumber("Left distance", getLeftDistance());
		SmartDashboard.putNumber("Right distance", getRightDistance());
		SmartDashboard.putNumber("Left speed", getLeftVelocity());
		SmartDashboard.putNumber("Right speed", getRightVelocity());
	}
	@Override
	public void periodic() {
		updateOdometry();
        m_fieldSim.setRobotPose(odometry.getPoseMeters());
	}

	@Override
	public void simulationPeriodic() {
		log();
	}
	
	public void drive(double xSpeed, double rot) {
		var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
		setSpeeds(wheelSpeeds);
	}
	public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
		final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
		final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

		final double leftOutput = leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
		final double rightOutput = rightPIDController.calculate(rightEncoder.getRate(),speeds.rightMetersPerSecond);
		leftMotor.set(leftOutput + leftFeedforward);
		rightMotor.set(rightOutput + rightFeedforward);
	}

/** Updates the field-relative position. */
	public void updateOdometry() {
		double l=leftEncoder.getDistance();
		double r=rightEncoder.getDistance();
		//System.out.println(l+" "+r);
		odometry.update(
			gyro.getRotation2d(), l, r);
	}
	public void resetOdometry(Pose2d pose) {
		leftEncoder.reset();
		rightEncoder.reset();
		odometry.resetPosition(pose, gyro.getRotation2d());
	}

    public Pose2d getPose() {
		//updateOdometry();
        return odometry.getPoseMeters();
    }

    public void odometryDrive(double xSpeed, double rot) {
		drive(xSpeed, rot);
		log();
	}
}
