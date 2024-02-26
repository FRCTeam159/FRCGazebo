// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
import frc.robot.Constants;
import frc.robot.objects.SwerveModule;
import gazebo.SimGyro;
import subsystems.Simulation;

public class Drivetrain extends SubsystemBase implements Constants {

	// square frame geometry

	static public boolean debug=false;
	
	public static final double kFrontWheelBase = Units.inchesToMeters(29); // distance beteen front wheels
	public static final double kSideWheelBase = Units.inchesToMeters(29); // distance beteen side wheels
	public static final double kTrackRadius = 0.5* (Math.sqrt(kFrontWheelBase*kFrontWheelBase+kSideWheelBase*kSideWheelBase));

	public static final double kRobotLength = Units.inchesToMeters(31); // Waffle side length
 
	public static double kMaxVelocity = 2; // meters per second
	public static double kMaxAcceleration = 1; // meters/second/second
	public static double kMaxAngularVelocity = Math.toRadians(720); // degrees per second
	public static double kMaxAngularAcceleration = Math.toRadians(360);// degrees per second per second

	public static double kMaxVelocityObserved=0;
	public static double kMaxAccelerationObserved=0;

	public static double dely = 0.5 * kSideWheelBase; // 0.2949 metters
	public static double delx = 0.5 * kFrontWheelBase;

	private final Translation2d m_frontLeftLocation = new Translation2d(delx, dely);
	private final Translation2d m_frontRightLocation = new Translation2d(delx, -dely);
	private final Translation2d m_backLeftLocation = new Translation2d(-delx, dely);
	private final Translation2d m_backRightLocation = new Translation2d(-delx, -dely);

	private final SwerveModule m_frontLeft = new SwerveModule(FL_DRIVE, FL_TURN,FL_ID);
	private final SwerveModule m_frontRight = new SwerveModule(FR_DRIVE, FR_TURN,FR_ID);
	private final SwerveModule m_backLeft = new SwerveModule(BL_DRIVE, BL_TURN,BL_ID);
	private final SwerveModule m_backRight = new SwerveModule(BR_DRIVE, BR_TURN,BR_ID); 

	private final SwerveModule[] modules={m_frontLeft,m_frontRight,m_backLeft,m_backRight};

	private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	private final SwerveModulePosition[] m_positions={
		new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()
	};

	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,new Rotation2d(), m_positions,new Pose2d());
	private Simulation simulation;

	public SimGyro m_gyro = new SimGyro(0);

	boolean field_oriented = false;
	boolean alligning=false;
	double last_heading = 0;
	Pose2d field_pose;
	Pose2d m_pose;

	boolean robot_disabled=true;
	boolean m_showtags=false;
	static boolean simstarted=false;

	boolean m_disabled = true;
	boolean m_resetting=false;
	
	private final Timer m_timer = new Timer();

    /** Creates a new Subsystem. */
	public Drivetrain() {
		m_timer.start();
		TargetMgr.init();

		simulation = new Simulation();
			
		// Configure AutoBuilder last
    	AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(3, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2, 0.0, 0.0), // Rotation PID constants
                    1, // Max module speed, in m/s
                    kTrackRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            //   var alliance = DriverStation.getAlliance();
            //   if (alliance.isPresent()) {
            //     return alliance.get() == DriverStation.Alliance.Red;
            //   }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
	}

	public void setRobotDisabled(boolean f){
		robot_disabled=f;
	}
	public double getTime() {
		//return simulation.getSimTime();
		return simulation.getSimTime();
	}

	public double getClockTime() {
		return Timer.getFPGATimestamp();
	}
	public void startAuto() {
		System.out.println("START AUTO");
		simulation.run();
		simulation.startAuto();
		enable();
	}

	public void endAuto() {
		System.out.println("END AUTO");
		if(debug)
			System.out.println("Drivetrain.endAuto");
		simulation.endAuto();
		
	}
	public void init() {
		if(debug)
			System.out.println("Drivetrain.init");
		field_pose = getPose();
		
		simulation.init();
		enable();
	}
	public boolean showTags() {
		return  m_showtags;
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
		m_gyro.disable();
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
		m_gyro.enable();
	}

	public void reset() {
		if(debug)
			System.out.println("Drivetrain.reset");
		for (int i = 0; i < modules.length; i++) {
			modules[i].reset();
		}
		m_gyro.reset();
		last_heading = 0;
		resetPositions();
	}

	public void setPose(Pose2d p) {
		//reset();
		last_heading = 0;
		resetOdometry(p);
		field_pose = getPose();
	}
	public void resetPose() {
		reset();
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
		field_oriented=t;
	}

	public boolean fieldOriented(){
		return field_oriented;
	}
	public boolean isGyroEnabled(){
		return true;
	}
	public double getHeading() {
		return getRotation2d().getDegrees();
	}

	public Rotation2d getRotation2d() {
		double angle =  m_gyro.getHeading();
		angle = unwrap(last_heading, angle);
		last_heading = angle;
		return Rotation2d.fromDegrees(angle);
	}

	public double getAngle() {
		return m_frontLeft.getAngle();
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
		if(!simstarted)
			return;
		Pose2d pose = getPose();
		String s = String.format(" X:%-5.2f Y:%-5.2f H:%-4.1f",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    	SmartDashboard.putString("Pose", s);

		double v=getVelocity();
		kMaxVelocityObserved=v>kMaxVelocityObserved?v:kMaxVelocityObserved;
		//SmartDashboard.putNumber("maxV", kMaxVelocityObserved);
		field_oriented = SmartDashboard.getBoolean("Field Oriented", field_oriented);
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
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxVelocity);
		for (int i = 0; i < modules.length; i++){
			modules[i].setDesiredState(swerveModuleStates[i]);
		}
		updateOdometry();
	}
	 // reset wheels turn motor to starting position
	public void resetWheels(boolean begin) {
		m_resetting = begin;
		if (begin) {
			System.out.println("Drivetrain-START ALIGNING_WHEELS");
			m_timer.reset();
		}
		//else
		//	System.out.println("Drivetrain-END ALIGNING_WHEELS time="+m_timer.get());
		for (int i = 0; i < modules.length; i++)
			modules[i].resetWheel(begin);
	}
	public void  alignWheels() {
		for (int i = 0; i < modules.length; i++)
			modules[i].alignWheel();
	}
	public boolean wheelsReset() {
		for (int i = 0; i < modules.length; i++) {
			if (!modules[i].wheelReset())
				return false;
		}
		if(m_resetting)
			System.out.println("Drivetrain-WHEELS-ARE-ALIGNED");
		m_resetting = false;
		return true;
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
		updatePositions();

		m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
		m_odometry.resetPosition(getRotation2d(), m_positions,pose);

		System.out.println("reset odometry:"+getPose());
		updateOdometry();
	}

	public SwerveDriveKinematics getKinematics(){
		return m_kinematics;
	}
	public Pose2d getPose() {
		return m_pose;
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

	private ChassisSpeeds getRobotRelativeSpeeds() {
		return m_kinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
				m_backRight.getState());
	}
	private void driveRobotRelative(ChassisSpeeds speed){
		this.drive(speed.vxMetersPerSecond, speed.vyMetersPerSecond, speed.omegaRadiansPerSecond, false);
	}
	public void resetPose(Pose2d pose){
		m_odometry.resetPosition(getRotation2d(), m_positions, pose);
	}
	
	public static boolean simStarted(){
		return simstarted;
	}
	@Override
	public void periodic() {
		//updateOdometry();
	}

	@Override
	public void simulationPeriodic() {
		if(!simstarted && simulation.started()){
			reset();
			simstarted=true;
		}
		if(robot_disabled)
			drive(0.0,0.0,0,false); // stay in place
		updateOdometry();
		log();
	}
}
