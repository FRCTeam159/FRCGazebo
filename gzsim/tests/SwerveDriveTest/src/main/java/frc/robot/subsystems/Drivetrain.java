// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.SwerveModule;
import gazebo.SimGyro;

public class DriveTrain extends SubsystemBase {

	// square frame geometry

	public static double front_wheel_base = 23.22; // distance beteen front wheels
	public static double side_wheel_base = 23.22; // distance beteen side wheels

	public static double dely = i2M(0.5 * side_wheel_base); // 0.2949 metters
	public static double delx = i2M(0.5 * front_wheel_base);

	private final Translation2d m_frontLeftLocation = new Translation2d(dely, -delx);
	private final Translation2d m_frontRightLocation = new Translation2d(dely, delx);
	private final Translation2d m_backLeftLocation = new Translation2d(-dely, -delx);
	private final Translation2d m_backRightLocation = new Translation2d(-dely, delx);

	private final SwerveModule m_frontLeft = new SwerveModule(1, 2);
	private final SwerveModule m_frontRight = new SwerveModule(3, 4);
	private final SwerveModule m_backLeft = new SwerveModule(5, 6);
	private final SwerveModule m_backRight = new SwerveModule(7, 8);

	private Simulation simulation;

	public SimGyro m_gyro = new SimGyro(0);

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

	public static final double kTrackWidth = i2M(2 * front_wheel_base); // bug? need to double actual value for geometry to work
	public static final double kWheelDiameter = i2M(4); // wheel diameter in tank model

	public static double kMaxVelocity = 3; // meters per second
	public static double kMaxAcceleration = 1; // meters/second/second
	public static double kMaxAngularSpeed = Math.toRadians(360); // degrees per second
	public static double kMaxAngularAcceleration = Math.toRadians(20);// degrees per second per second

	public boolean enable_gyro = true;

	private double last_heading = 0;

	private Pose2d field_pose;

	boolean m_disabled = true;

	/** Creates a new Subsystem. */
	public DriveTrain() {
		simulation = new Simulation(this);
		SmartDashboard.putBoolean("Field Oriented", enable_gyro);
		SmartDashboard.putNumber("maxV", kMaxVelocity);
		SmartDashboard.putNumber("maxA", kMaxAcceleration);
	}

	private static double i2M(double inches) {
		return inches * 0.0254;
	}

	public double getTime() {
		return simulation.getSimTime();
	}

	public void startAuto() {
		simulation.reset();
		simulation.start();
		enable();
	}

	public void endAuto() {
		//simulation.end();
		disable();
	}
	public void init() {
		System.out.println("Drivetrain.init");
		field_pose = getPose();
		simulation.init();
		enable();
	}

	public boolean enabled(){
		return !m_disabled;
	}
	public boolean disabled(){
		return m_disabled;
	}
	public void disable() {
		m_disabled = true;
		System.out.println("Drivetrain.disable");
		m_frontLeft.disable();
		m_frontRight.disable();
		m_backLeft.disable();
		m_backRight.disable();
		m_gyro.disable();
		simulation.end();
	}

	public void enable() {
		m_disabled = false;
		simulation.run();
		System.out.println("Drivetrain.enable");
		m_frontLeft.enable();
		m_frontRight.enable();
		m_backLeft.enable();
		m_backRight.enable();
		m_gyro.enable();
	}

	public void reset() {
		m_disabled = true;
		field_pose = add(field_pose, getPose());
		System.out.println("Drivetrain.reset");
		m_frontLeft.reset();
		m_frontRight.reset();
		m_backLeft.reset();
		m_backRight.reset();

		m_gyro.reset();
		last_heading = 0;
		//Pose2d pose=new Pose2d(0,0,new Rotation2d(0));
		//m_odometry.resetPosition(pose, new Rotation2d(0));
	}

	public void resetPose() {
		//m_gyro.reset();

		last_heading = 0;
		resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		field_pose = getPose();
	}

	// get transform from pose
	public static Transform2d getTransform(Pose2d pose) {
		return new Transform2d(pose.getTranslation(), pose.getRotation());
	}

	// add two poses
	public static Pose2d add(Pose2d p1, Pose2d p2) {
		Transform2d td = getTransform(p2);
		return p1.plus(td);
	}

	public void setFieldOriented(boolean t){
		m_gyro.setEnabled(t);
	}

	public boolean isGyroEnabled(){
		return enable_gyro;
	}
	public double getHeading() {
		return getRotation2d().getDegrees();
	}

	public double gyroHeading() {
		return m_gyro.getHeading();
	}

	public Rotation2d getRotation2d() {
		double angle = gyroHeading();
		angle = unwrap(last_heading, angle);
		last_heading = angle;
		return Rotation2d.fromDegrees(angle);
	}

	public double getLeftDistance() {
		return 0.5*(m_frontLeft.getDistance()+m_backLeft.getDistance());
	}

	public double getRightDistance() {
		return 0.5*(m_frontRight.getDistance()+m_backRight.getDistance());
	}

	public double getLeftVelocity() {
		return 0.5*(m_frontLeft.getMoveRate()+m_backLeft.getMoveRate());
	}

	public double getRightVelocity() {
		return 0.5*(m_frontRight.getMoveRate()+m_backRight.getMoveRate());
	}

	public double getVelocity() {
		return 0.5 * (getLeftVelocity() + getRightVelocity());
	}

	public double getDistance() {
		return 0.5 * (getLeftDistance() + getRightDistance());
	}

	public void log() {
		SmartDashboard.putNumber("Heading", getHeading());
		SmartDashboard.putNumber("Distance", getDistance());
		SmartDashboard.putNumber("Velocity", getVelocity());
		Translation2d t=  getPose().getTranslation();
		double x=t.getX();
		double y=t.getY();
		SmartDashboard.putNumber("X:", x);
		SmartDashboard.putNumber("Y:", y);

		//SmartDashboard.putString("Location", getPose().getTranslation().toString());
		enable_gyro = SmartDashboard.getBoolean("Field Oriented", enable_gyro);
		kMaxVelocity=SmartDashboard.getNumber("maxV", kMaxVelocity);
		kMaxAcceleration=SmartDashboard.getNumber("maxA", kMaxAcceleration);
	}

	@Override
	public void periodic() {
		//updateOdometry();
	}

	@Override
	public void simulationPeriodic() {
		if(!enabled()){
			//move(0);
			//turn(0);
			drive(0.0,0.0,0.0,false); // stay in place
		}
		updateOdometry();
		log();
	}

	public void turn(double value) {
		m_frontLeft.turn(value);
		m_frontRight.turn(value);
		m_backLeft.turn(value);
		m_backRight.turn(value);
	}

	public void move(double value) {
		m_frontLeft.move(value);
		m_frontRight.move(value);
		m_backLeft.move(value);
		m_backRight.move(value);
	}

	public void testDrive(double speed, double rot) {
		turn(rot);
		move(speed);
	}

	public void turnInPlace(double value) {
		m_frontLeft.setAngle(45, value);
		m_frontRight.setAngle(-45, -value);
		m_backLeft.setAngle(-45, value);
		m_backRight.setAngle(45, -value);
		updateOdometry();
		//drive(0, 0, value, true);
	}

	public void driveForward(double value) {
		m_frontLeft.setAngle(0, value);
		m_frontRight.setAngle(0, value);
		m_backLeft.setAngle(0, value);
		m_backRight.setAngle(0, value);
		updateOdometry();
	}
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates = m_kinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxVelocity);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);
		updateOdometry();
	}

	// A teleop drive method that uses odometry and kinetemtics
    public void odometryDrive(double xSpeed, double rot) {
		drive(xSpeed, 0, rot, false);
	}
	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		field_pose = m_odometry.update(
				m_gyro.getRotation2d(),
				m_frontLeft.getState(),
				m_frontRight.getState(),
				m_backLeft.getState(),
				m_backRight.getState());
		log();
	}

	public void resetOdometry(Pose2d pose) {
		reset();
		last_heading = 0;
		m_gyro.reset();
		m_odometry.resetPosition(pose, getRotation2d());
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public Pose2d getFieldPose() {
		return field_pose;
	}

	// removes heading discontinuity at 180 degrees
	public static double unwrap(double previous_angle, double new_angle) {
		double d = new_angle - previous_angle;
		d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
		return previous_angle + d;
	}

}
