// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.SwerveModule;
import gazebo.SimGyro;

public class Drivetrain extends SubsystemBase {

    // square frame geometry
	private final Translation2d m_frontLeftLocation = new Translation2d(0.381, -0.381);
	private final Translation2d m_frontRightLocation = new Translation2d(0.381, 0.381);
	private final Translation2d m_backLeftLocation = new Translation2d(-0.381, -0.381);
	private final Translation2d m_backRightLocation = new Translation2d(-0.381, 0.381);
  
	private final SwerveModule m_frontLeft = new SwerveModule(1, 2);
    private final SwerveModule m_frontRight = new SwerveModule(3, 4);
    private final SwerveModule m_backLeft = new SwerveModule(5, 6);
    private final SwerveModule m_backRight = new SwerveModule(7, 8);

	private Simulation simulation;

	public SimGyro m_gyro = new SimGyro(0);

	private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  	private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

	public static final double kTrackWidth = i2M(2*23); // bug? need to double actual value for geometry to work
	public static final double kWheelDiameter = i2M(4); // wheel diameter in tank model
	
	public static double kMaxVelocity = 4; // meters per second
	public static double kMaxAcceleration = 1; //  meters/second/second
	public static double kMaxAngularSpeed =  Math.toRadians(90); //degrees per second
	public static double kMaxAngularAcceleration = Math.toRadians(10);// degrees per second per second

	private final DifferentialDriveOdometry odometry;

	public boolean enable_gyro = true;

	private double last_heading=0;

	private Pose2d field_pose;

	boolean m_disabled=true;
	
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

		//leftMotor.setScale(scale);
		//rightMotor.setScale(scale);

		odometry = new DifferentialDriveOdometry(getRotation2d());
		
		SmartDashboard.putBoolean("Enable gyro", enable_gyro);
	}

	private static double i2M(double inches) {
		return inches * 0.0254;
	}
	private double r2D(double radians) {
		return  180*radians/Math.PI;
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
		field_pose=getPose();
		simulation.init();
		enable();
	}

	public void disable(){
		m_disabled=true;
		System.out.println("Drivetrain.disable");
		m_frontLeft.disable();
		m_frontRight.disable();
		m_backLeft.disable();
		m_backRight.disable();
		m_gyro.disable();
		simulation.end();
	}
	public void enable(){
		m_disabled=false;
		simulation.run();
		System.out.println("Drivetrain.enable");
		m_frontLeft.enable();
		m_frontRight.enable();
		m_backLeft.enable();
		m_backRight.enable();
		m_gyro.enable();
	}
	public void reset(){
		m_disabled=true;
		field_pose=add(field_pose,getPose());
		System.out.println("Drivetrain.reset");
		m_frontLeft.reset();
		m_frontRight.reset();
		m_backLeft.reset();
		m_backRight.reset();

		m_gyro.reset();
		last_heading =0;
	}
	public void resetPose() {
		resetOdometry(new Pose2d(0,0,new Rotation2d(0)));
		field_pose=getPose();
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
	public double getHeading(){
		return getRotation2d().getDegrees();
	}
	
	public double gyroHeading(){
		return m_gyro.getHeading();
	}
	public double calcHeading(){
		double angle=r2D((getRightDistance()-getLeftDistance())/kTrackWidth);	
		return angle;
	}
	
	public Rotation2d getRotation2d(){
		double angle=gyroHeading();
		angle=unwrap(last_heading,angle);
		last_heading=angle;
		return Rotation2d.fromDegrees(angle);
	}
	public double getLeftDistance(){
		return  m_frontLeft.getDistance();
	}
	public double getRightDistance(){
		return  m_frontRight.getDistance();
	}
	public double getLeftVelocity(){
		return  m_frontLeft.getMoveRate();
	}
	public double getRightVelocity(){
		return  m_frontRight.getMoveRate();
	}
	public double getVelocity(){
		return  0.5*(getLeftVelocity()+getRightVelocity());
	}
	public double getDistance(){
		return  0.5*(getLeftDistance()+getRightDistance());
	}
	public void log(){
		SmartDashboard.putNumber("Heading", getHeading());
		SmartDashboard.putNumber("Distance", getDistance());
		SmartDashboard.putNumber("Velocity", getVelocity());
		enable_gyro=SmartDashboard.getBoolean("Enable gyro", enable_gyro); 
	}
	@Override
	public void periodic() {
		updateOdometry();
	}

	@Override
	public void simulationPeriodic() {
		updateOdometry();
		log();
	}
	
	public void turn(double value){
		m_frontLeft.turn(value);
		m_frontRight.turn(value);
		m_backLeft.turn(value);
		m_backRight.turn(value);
	}
	public void move(double value){
		m_frontLeft.move(value);
		m_frontRight.move(value);
		m_backLeft.move(value);
		m_backRight.move(value);
	}
	public void testDrive(double speed, double rot) {
		turn(rot);
		move(speed);
	}
	
	public void turnInPlace(double value){
		m_frontLeft.setAngle(45,value);
		m_frontRight.setAngle(-45,-value);
		m_backLeft.setAngle(-45,value);
		m_backRight.setAngle(45,-value);
	}
	@SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
	if(m_disabled)
	  return;
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxVelocity);
	//for(int i=0;i<4;i++)
	//	System.out.println(swerveModuleStates[i]);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    updateOdometry();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    field_pose=m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
        log();
  }
    /** Updates the field-relative position. */
	// public void updateOdometry() {
	// 	double l=m_frontLeft.getDistance();
	// 	double r=m_frontRight.getDistance();
	// 	odometry.update(getRotation2d(), l, r);
	// }
	public void resetOdometry(Pose2d pose) {
		reset();
		last_heading = 0;
		m_gyro.reset();
		odometry.resetPosition(pose, getRotation2d());
	}
    public Pose2d getPose() {
        return odometry.getPoseMeters();
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
