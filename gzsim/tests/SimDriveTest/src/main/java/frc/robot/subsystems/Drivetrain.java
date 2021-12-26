// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

	private static final double kTrackWidth = i2M(28.25); // inches
	private static final double kWheelRadius = i2M(4); // 8 inch wheel diameter in tank model

	/** Creates a new Subsystem. */
	public Drivetrain() {
		leftMotor = new SimMotor(FRONT_LEFT);
		rightMotor = new SimMotor(FRONT_RIGHT);
		leftEncoder=new SimEncoder(FRONT_LEFT);
		rightEncoder=new SimEncoder(FRONT_RIGHT);
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
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		log();
	}
}
