// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.objects.SwerveModule;
import gazebo.SimGyro;
import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {

	// square frame geometry

	static public boolean debug=true;
	static public boolean debug_angles=true;

	public static double dely = Units.inchesToMeters(0.5 * kFrontWheelBase); // 0.2949 metters
	public static double delx = Units.inchesToMeters(0.5 * kSideWheelBase);

	private final Translation2d m_frontLeftLocation = new Translation2d(delx, dely);
	private final Translation2d m_frontRightLocation = new Translation2d(delx, -dely);
	private final Translation2d m_backLeftLocation = new Translation2d(-delx, dely);
	private final Translation2d m_backRightLocation = new Translation2d(-delx, -dely);

	private final SwerveModule m_frontLeft = new SwerveModule(kFl_Drive, kFl_Turn);
	private final SwerveModule m_frontRight = new SwerveModule(kFr_Drive, kFr_Turn);
	private final SwerveModule m_backLeft = new SwerveModule(kBl_Drive, kBl_Turn);
	private final SwerveModule m_backRight = new SwerveModule(kBr_Drive, kBr_Turn); 
	
	private final SwerveModule[] m_modules={
		m_frontLeft,m_frontRight,m_backLeft,m_backRight
	};
	private final SwerveModulePosition[] m_positions={
		new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()
	};

	private Simulation simulation;
	public SimGyro m_gyro = new SimGyro(0);

	private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	public static final double kTrackWidth = Units.inchesToMeters(2 * kFrontWheelBase); // bug? need to double actual value for geometry to work

	boolean field_oriented = true;
	double last_heading = 0;
	Pose2d field_pose;

	boolean robot_disabled=true;

	double x_std=0.1;
	double y_std=0.1;
	double h_std=1.0;

	double latency=0.05;
	double vision_confidence=0.0;
	boolean use_tags=false;

	boolean m_disabled = true;
	SwerveDrivePoseEstimator m_poseEstimator;

	private final Timer m_timer = new Timer();

	private int cnt=0;

	Pose2d vision_pose;

    /** Creates a new Subsystem. */
	public Drivetrain() {
		m_timer.start();

		makeEstimator();

		TargetMgr.init();
		simulation = new Simulation(this);
		SmartDashboard.putBoolean("Field Oriented", field_oriented);
		SmartDashboard.putNumber("maxV", kMaxVelocity);
		SmartDashboard.putNumber("maxA", kMaxAcceleration);
		SmartDashboard.putBoolean("Use Tags", use_tags);
		SmartDashboard.putNumber("Latency", latency);
		SmartDashboard.putNumber("Conf", vision_confidence);
		SmartDashboard.putString("State","Driving");
	}

	void makeEstimator(){
		double vmult=1.0/vision_confidence;
		SwerveModulePosition[] positions={
			new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()
		};
		m_poseEstimator=new SwerveDrivePoseEstimator (
          m_kinematics,
          new Rotation2d(),
          positions,
          new Pose2d(),
          VecBuilder.fill(x_std, y_std, Units.degreesToRadians(h_std)), // encoder accuracy
          VecBuilder.fill(vmult*x_std, vmult*y_std, Units.degreesToRadians(vmult*h_std))); // encoder accuracy
	}

	public double getTime() {
		return simulation.getSimTime();
	}

	public double getClockTime() {
		//return simulation.getClockTime();
		return Timer.getFPGATimestamp();
	}
	public void startAuto() {
		simulation.reset();
		simulation.start();
		enable();
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
	public void disabledInit() {
		simulation.end();
	}
	
	public void enable() {
		m_disabled = false;
		//simulation.run();
		if(debug)
		System.out.println("Drivetrain.enable");
		for(int i=0;i<m_modules.length;i++)
			m_modules[i].enable();	
		m_gyro.enable();
	}

	public void disable() {
		m_disabled = true;
		if(debug)
			System.out.println("Drivetrain.disable");
		for(int i=0;i<m_modules.length;i++)
			m_modules[i].disable();
		m_gyro.disable();
		simulation.end();
	}

	public void reset() {
		m_disabled = true;
		if(debug)
			System.out.println("Drivetrain.reset");
		for(int i=0;i<m_modules.length;i++)
			m_modules[i].reset();	
		TargetMgr.clearStartPose();
		last_heading =0;
		m_gyro.reset();
		makeEstimator();
	}

	public void resetPose() {
		resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		// Pose2d p=new Pose2d();	
		// last_heading = p.getRotation().getDegrees();
		// resetOdometry(p);
		 field_pose = getPose();
	}

	public void setUseTags(boolean b){
		use_tags=b;
	}
	public boolean useTags(){
		return use_tags;
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

	public boolean isFieldOriented(){
		return field_oriented;
	}
	public double getHeading() {
	    return getRotation2d().getDegrees();
	}

	public Rotation2d gyroRotation2d() {
		return m_gyro.getRotation2d();
	}

	public Rotation2d getRotation2d() {
		double angle =m_gyro.getHeading(); // good auto
		angle = unwrap(last_heading, angle);
		last_heading = angle;
		return Rotation2d.fromDegrees(angle);
	}
	public double lastHeading(){
		return last_heading;
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
		SmartDashboard.putNumber("Velocity", getVelocity());
		Translation2d t=  getPose().getTranslation();
		
		SmartDashboard.putNumber("H:", getHeading());
		SmartDashboard.putNumber("X:", t.getX());
		SmartDashboard.putNumber("Y:", t.getY());

		use_tags = SmartDashboard.getBoolean("Use Tags", use_tags);

		field_oriented = SmartDashboard.getBoolean("Field Oriented", field_oriented);
		TargetMgr.setFieldRelative(field_oriented);
		kMaxVelocity=SmartDashboard.getNumber("maxV", kMaxVelocity);
		kMaxAcceleration=SmartDashboard.getNumber("maxA", kMaxAcceleration);

		latency=SmartDashboard.getNumber("Latency", 0);
		double conf=SmartDashboard.getNumber("Conf", vision_confidence);
		if(vision_pose !=null){
			String s=String.format("X:%-3.2f Y:%-3.2f",vision_pose.getX(),vision_pose.getY());
			SmartDashboard.putString("Vision",s);
		}
		if(Math.abs(conf-vision_confidence)>1e-7){
			vision_confidence=conf;
			double vmult=1.0/vision_confidence;
			m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(vmult*x_std, vmult*y_std, Units.degreesToRadians(vmult*h_std)));
		}
		String str=String.format("Angles fl:%-1.2f fr:%-1.2f bl:%-1.2f br:%-1.2f\n",
			m_frontLeft.getAngle(),m_frontRight.getAngle(),m_backLeft.getAngle(),m_backRight.getAngle());
			SmartDashboard.putString("Wheel", str);		
	}

	@Override
	public void periodic() {
		if (Robot.isRobotDisabled())
			drive(0.0, 0.0, 0, true); // stay in place
		updateOdometry();
		log();
	}

	@Override
	public void simulationPeriodic() {}

	void displayAngles(){
		if((cnt%100)==0){
			String str=String.format("angles fl:%-1.2f fr:%-1.2f bl:%-1.2f br:%-1.2f\n",
			m_frontLeft.getAngle(),m_frontRight.getAngle(),m_backLeft.getAngle(),m_backRight.getAngle());
			SmartDashboard.putString("Wheel ", str);
		}
		cnt++;
	}
	
	public void turn(double value) {
		for(int i=0;i<m_modules.length;i++)
			m_modules[i].turn(value);
	}

	public void move(double value) {
		for(int i=0;i<m_modules.length;i++)
			m_modules[i].move(value);
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
		m_frontLeft.setAngle(0, value);
		m_frontRight.setAngle(0, value);
		m_backLeft.setAngle(0, value);
		m_backRight.setAngle(0, value);
		updateOdometry();
	}
	public double getXVoltage(){
		return 0.5*(m_frontLeft.getXVoltage()+m_frontRight.getXVoltage());
	}
	public double getYVoltage(){
		return 0.5*(m_frontLeft.getYVoltage()+m_backLeft.getYVoltage());
	}
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		SwerveModuleState swerveModuleStates[] = m_kinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxVelocity);
		for(int i=0;i<m_modules.length;i++)
			m_modules[i].setDesiredState(swerveModuleStates[i]);
		
		updateOdometry();
	}

	// A teleop drive method that uses odometry and kinematics
    public void odometryDrive(double xSpeed, double rot) {
		drive(xSpeed, 0, rot, false);
	}
	

	public void updatePositions(){
		for(int i=0;i<m_modules.length;i++)
			m_positions[i]=m_modules[i].getPosition();
	}
	public void resetPositions(){
		for(int i=0;i<m_positions.length;i++)
			m_positions[i]=new SwerveModulePosition();
	}
	public void resetOdometry() {
		resetOdometry(new Pose2d());
	}
	public void resetOdometry(Pose2d pose) {
		last_heading = 0;
		m_gyro.reset();
		resetPositions();
		m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
		m_poseEstimator.resetPosition(gyroRotation2d(), m_positions,pose);

		System.out.println("reset odometry:"+getPose());
		updateOdometry();
	}

	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		updatePositions();
		//m_poseEstimator.update(gyroRotation2d(), m_positions);
		m_poseEstimator.updateWithTime(getTime(),gyroRotation2d(), m_positions);
		// apply vision measurements - this must be calculated based either on latency or timestamps.
		vision_pose=TagDetector.getLastPose();
		if(vision_pose !=null&& use_tags && vision_confidence>0){
			double tm=getTime()-latency;
			if(tm>0.1)
				m_poseEstimator.addVisionMeasurement(vision_pose,tm);
		}											
		field_pose = getPose();
	}
	public Pose2d getPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	public Pose2d getFieldPose() {
		return field_pose;
	}
	
	// removes heading discontinuity at 180 degrees
	public static double unwrap(double previous_angle, double new_angle) {
		double d = new_angle - previous_angle%360;
		d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
		return previous_angle + d;
	}

	public static boolean PConTarget(ProfiledPIDController ppc, double perr){
		double verr=ppc.getVelocityError();
		double ptol=ppc.getPositionTolerance();
		double vtol=ppc.getVelocityTolerance();
		if(Math.abs(perr)<ptol && Math.abs(verr)<vtol)
			return true;
		return false;
	}
}
