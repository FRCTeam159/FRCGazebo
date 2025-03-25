// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import simreal.Gyro;
import subsystems.Simulation;

public class Drivetrain extends SubsystemBase {

	// square frame geometry

	static public boolean debug=true;
	static public boolean debug_angles=false;
	static boolean m_resetting = false;

	public static double front_wheel_base = 23.22; // distance beteen front wheels
	public static double side_wheel_base = 23.22; // distance beteen side wheels

	public static double dely = Units.inchesToMeters(0.5 * side_wheel_base); // 0.2949 metters
	public static double delx = Units.inchesToMeters(0.5 * front_wheel_base);

	private final Translation2d m_frontLeftLocation = new Translation2d(delx, dely);
	private final Translation2d m_frontRightLocation = new Translation2d(delx, -dely);
	private final Translation2d m_backLeftLocation = new Translation2d(-delx, dely);
	private final Translation2d m_backRightLocation = new Translation2d(-delx, -dely);

	private final SwerveModule m_frontLeft = new SwerveModule(kFl_Drive, kFl_Turn, 1);
    private final SwerveModule m_frontRight = new SwerveModule(kFr_Drive, kFr_Turn, 2);
    private final SwerveModule m_backLeft = new SwerveModule(kBl_Drive, kBl_Turn, 4);
    private final SwerveModule m_backRight = new SwerveModule(kBr_Drive, kBr_Turn, 3);
   	
	public static String chnlnames[] = { "BR", "BL", "FL", "FR" };

	private final SwerveModulePosition[] m_positions={
		new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()
	};
	private final SwerveModule[] modules={
		m_frontLeft,m_frontRight,m_backLeft,m_backRight
	};

	private Simulation simulation=null;

	public Gyro m_gyro = new Gyro();

	private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,new Rotation2d(), m_positions,new Pose2d());

	public static final double kTrackWidth = Units.inchesToMeters(2 * front_wheel_base); // bug? need to double actual value for geometry to work

	public static double kMaxVelocity = 3; // meters per second
	public static double kMaxAcceleration = 0.5; // meters/second/second
	public static double kMaxAngularVelocity = Math.toRadians(360); // degrees per second
	public static double kMaxAngularAcceleration = Math.toRadians(90);// degrees per second per second

	boolean m_field_oriented = false;
	double last_heading = 0;
	Pose2d field_pose;
	Pose2d m_pose;

	boolean robot_disabled=true;
	boolean m_disabled = true;
	
	private int cnt=0;

    /** Creates a new Subsystem. */
	public Drivetrain() {
		simulation = new Simulation();
		SmartDashboard.putBoolean("Field Oriented", m_field_oriented);
		SmartDashboard.putNumber("maxV", kMaxVelocity);
		SmartDashboard.putNumber("maxA", kMaxAcceleration);
	}

	public double getTime() {
		return simulation.getSimTime();
	}

	public double getClockTime() {
		return Timer.getFPGATimestamp();
	}
	public void startAuto() {
		simulation.reset();
		simulation.start();
		enable();
	}

	public double getVelocity(){
		double speed=0;
		for(int i=0;i<modules.length;i++)
			speed+=modules[i].getVelocity();
		return speed/modules.length;
	}
	public double getDistance(){
		double distance=0;
		for(int i=0;i<modules.length;i++)
			distance+=modules[i].getDistance();
		return distance/modules.length;
	}
	public void endAuto() {
		if(debug)
			System.out.println("Drivetrain.endAuto");
	}
	public void init() {
		if(debug)
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
		if(debug)
			System.out.println("Drivetrain.disable");
		for (int i = 0; i < modules.length; i++) {
			modules[i].disable();
		}
		simulation.end();
	}

	public void enable() {
		m_disabled = false;
		simulation.run();
		if(debug)
			System.out.println("Drivetrain.enable");
		for (int i = 0; i < modules.length; i++) {
			modules[i].enable();
		}
	}

	public void reset() {
		m_disabled = true;
		if(debug)
			System.out.println("Drivetrain.reset");
		for (int i = 0; i < modules.length; i++) {
			modules[i].reset();
		}
	
		m_gyro.reset();
		last_heading = 0;
	}

	public void resetPose() {
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
		m_field_oriented=t;
	}

	public boolean isFieldOriented(){
		return m_field_oriented;
	}
	public double getHeading() {
		return getRotation2d().getDegrees();
	}

	public Rotation2d gyroRotation2d() {
		return Rotation2d.fromDegrees(m_gyro.getAngle());
	}
	public double gyroHeading() {
		return m_gyro.getAngle();
	}

	public Rotation2d getRotation2d() {
		double angle = gyroHeading();
		angle = unwrap(last_heading, angle);
		last_heading = angle;
		return Rotation2d.fromDegrees(angle);
	}

	public void log() {
		Pose2d pose = getPose();
		String s = String.format(" X:%-5.2f Y:%-5.2f H:%-4.1f D:%-3.1f V:%-3.1f",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees(),getDistance(),getVelocity());
    	SmartDashboard.putString("Pose", s);
	
		m_field_oriented = SmartDashboard.getBoolean("Field Oriented", m_field_oriented);
		kMaxVelocity=SmartDashboard.getNumber("maxV", kMaxVelocity);
		kMaxAcceleration=SmartDashboard.getNumber("maxA", kMaxAcceleration);
	
		if(debug_angles)
			displayAngles();	
	}

	@Override
	public void periodic() {
		boolean b = SmartDashboard.getBoolean("Reset", false);
		if (b && !m_resetting) { // start reset
			m_resetting = true;
		}
		else if (!b && m_resetting) { // end reset
			m_resetting = false;
			reset();
			resetPose();
		}
		updateOdometry();
		log();
	}

	void displayAngles(){
		if((cnt%100)==0){
			String str=String.format("angles fl:%-1.1f fr:%-1.1f bl:%-1.1f br:%-1.1f\n",
			m_frontLeft.getDegrees(),m_frontRight.getDegrees(),m_backLeft.getDegrees(),m_backRight.getDegrees());
			SmartDashboard.putString("Wheels ", str);
		}
		cnt++;
	}
	
	public void turn(double value) {
		for (int i = 0; i < modules.length; i++)
			modules[i].turn(value);
	}

	public void move(double value) {
		for (int i = 0; i < modules.length; i++)
			modules[i].move(value);
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
	}

	public void driveForward(double value) {
		for (int i = 0; i < modules.length; i++) 
			modules[i].setAngle(0, value);
		updateOdometry();
	}
	
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates = m_kinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,  gyroRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxVelocity);
		for (int i = 0; i < modules.length; i++){
			modules[i].setDesiredState(swerveModuleStates[i]);
		}
		updateOdometry();
	}

	// A teleop drive method that uses odometry and kinematics
    public void odometryDrive(double xSpeed, double rot) {
		drive(xSpeed, 0, rot, false);
	}
	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		updatePositions();
		m_pose=m_odometry.update(getRotation2d(), m_positions);
		field_pose = getPose();		
		log();
	}

	public void updatePositions(){
		for(int i=0;i<m_positions.length;i++)
			m_positions[i] = modules[i].getPosition();	  
	}

	public void resetPositions(){	
		for(int i=0;i<m_positions.length;i++)
			m_positions[i] = new SwerveModulePosition();	  
	}
	public void resetOdometry(Pose2d pose) {
		last_heading = 0;
		m_gyro.reset();
		resetPositions();
		m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
		m_odometry.resetPosition(getRotation2d(), m_positions,pose);
		System.out.println("reset odometry:"+getPose());
		updateOdometry();
	}

	public Pose2d getPose() {
		return m_pose;
	}

	public Pose2d getFieldPose() {
		return field_pose;
	}
	
	// reset wheels turn motor to starting position
	public void resetWheels(boolean begin) {
		if (begin) {
		  m_resetting = true;
		  System.out.println("Drivetrain-ALIGNING_WHEELS");
		}
		for (int i = 0; i < modules.length; i++) {
		  modules[i].resetWheel();
		}
	}
	public void  alignWheels() {
		for (int i = 0; i < modules.length; i++)
			modules[i].alignWheel();
	}
	
	// return true if all wheels are reset
	public boolean wheelsReset() {
		for (int i = 0; i < modules.length; i++) {
			if (!modules[i].wheelReset())
				return false;
		}
		if (m_resetting)
			System.out.println("Drivetrain-WHEELS_ALIGNED");
		m_resetting = false;
		return true;
	}

	// removes heading discontinuity at 180 degrees
	public static double unwrap(double previous_angle, double new_angle) {
		double d = new_angle - previous_angle;
		d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
		return previous_angle + d;
	}
}
