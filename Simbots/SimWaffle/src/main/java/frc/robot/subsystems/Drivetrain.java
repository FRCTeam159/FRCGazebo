// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.SwerveModule;
import gazebo.SimGyro;

public class Drivetrain extends SubsystemBase {

	// square frame geometry

	static public boolean debug=true;
	static public boolean debug_angles=false;

	public static double front_wheel_base = 23.22; // distance beteen front wheels
	public static double side_wheel_base = 23.22; // distance beteen side wheels

	public static double dely = Units.inchesToMeters(0.5 * side_wheel_base); // 0.2949 metters
	public static double delx = Units.inchesToMeters(0.5 * front_wheel_base);

	private final Translation2d m_frontLeftLocation = new Translation2d(delx, dely);
	private final Translation2d m_frontRightLocation = new Translation2d(delx, -dely);
	private final Translation2d m_backLeftLocation = new Translation2d(-delx, dely);
	private final Translation2d m_backRightLocation = new Translation2d(-delx, -dely);

	private final SwerveModule m_frontLeft = new SwerveModule(1, 2,1);
	private final SwerveModule m_frontRight = new SwerveModule(3, 4,2);
	private final SwerveModule m_backLeft = new SwerveModule(5, 6,3);
	private final SwerveModule m_backRight = new SwerveModule(7, 8,4); 
	
	//final PIDController m_controller=new PIDController(1,0.1,0.0);

	public static String chnlnames[] = { "BR", "BL", "FL", "FR" };

	private final SwerveModulePosition[] m_positions={
		new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()
	};

	//Translation2d cameraToRobotOffset=new Translation2d(Units.inchesToMeters(12.0),0);
    //Transform2d cameraToRobot=new Transform2d(cameraToRobotOffset,new Rotation2d());
	private Simulation simulation;

	public SimGyro m_gyro = new SimGyro(0);

	private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	public static final double kTrackWidth = Units.inchesToMeters(2 * front_wheel_base); // bug? need to double actual value for geometry to work

	public static double kMaxVelocity = 3; // meters per second
	public static double kMaxAcceleration = 0.5; // meters/second/second
	public static double kMaxAngularSpeed = Math.toRadians(360); // degrees per second
	public static double kMaxAngularAcceleration = Math.toRadians(90);// degrees per second per second

	boolean enable_gyro = true;
	double last_heading = 0;
	Pose2d field_pose;

	boolean robot_disabled=true;

	double x_std=0.1;
	double y_std=0.1;
	double h_std=5.0;

	double latency=0.05;
	double vision_confidence=0.00;
	double pose_error=0;
	boolean use_tags=false;

	boolean m_disabled = true;
	SwerveDrivePoseEstimator m_poseEstimator;

	private final Timer m_timer = new Timer();

	private int cnt=0;

    /** Creates a new Subsystem. */
	public Drivetrain() {
		m_timer.start();
		makeEstimator();

		TargetMgr.init();
		simulation = new Simulation(this);
		SmartDashboard.putBoolean("Field Oriented", enable_gyro);
		SmartDashboard.putNumber("maxV", kMaxVelocity);
		SmartDashboard.putNumber("maxA", kMaxAcceleration);
		SmartDashboard.putBoolean("Use Tags", use_tags);
		SmartDashboard.putNumber("Latency", latency);
		SmartDashboard.putNumber("Conf", vision_confidence);
		SmartDashboard.putNumber("Error", pose_error);
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

	public void setRobotDisabled(boolean f){
		robot_disabled=f;
	}
	public double getTime() {
		return simulation.getSimTime();
	}

	public double getClockTime() {
		//return simulation.getClockTime();
		return Timer.getFPGATimestamp();
		//return m_timer.get();
	}
	public void startAuto() {
		simulation.reset();
		simulation.start();
		enable();
	}

	public void endAuto() {
		if(debug)
		System.out.println("Drivetrain.endAuto");
		setRobotDisabled(true);
		//simulation.end();
		//disable();
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
		if(debug)
		System.out.println("Drivetrain.enable");
		m_frontLeft.enable();
		m_frontRight.enable();
		m_backLeft.enable();
		m_backRight.enable();
		m_gyro.enable();
	}

	public void reset() {
		m_disabled = true;
		if(debug)
		System.out.println("Drivetrain.reset");
		m_frontLeft.reset();
		m_frontRight.reset();
		m_backLeft.reset();
		m_backRight.reset();

		m_gyro.reset();
		last_heading = 0;
		//makeEstimator();
	}

	public void resetPose() {
		last_heading = 0;
		resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		field_pose = getPose();
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
		m_gyro.setEnabled(t);
	}

	public boolean isGyroEnabled(){
		return enable_gyro;
	}
	public double getHeading() {
		return getRotation2d().getDegrees();
	}

	public Rotation2d gyroRotation2d() {
		return m_gyro.getRotation2d();
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

		SmartDashboard.putNumber("Distance", getDistance());
		SmartDashboard.putNumber("Velocity", getVelocity());
		Translation2d t=  getPose().getTranslation();
		
		SmartDashboard.putNumber("H:", getHeading());
		SmartDashboard.putNumber("X:", t.getX());
		SmartDashboard.putNumber("Y:", t.getY());

		enable_gyro = SmartDashboard.getBoolean("Field Oriented", enable_gyro);
		kMaxVelocity=SmartDashboard.getNumber("maxV", kMaxVelocity);
		kMaxAcceleration=SmartDashboard.getNumber("maxA", kMaxAcceleration);

		use_tags = SmartDashboard.getBoolean("Use Tags", use_tags);
		SmartDashboard.putNumber("Error", pose_error);

		latency=SmartDashboard.getNumber("Latency", 0);
		double conf=SmartDashboard.getNumber("Conf", vision_confidence);
		if(Math.abs(conf-vision_confidence)>0.001){
			vision_confidence=conf;
			double vmult=1.0/vision_confidence;
			m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(vmult*x_std, vmult*y_std, Units.degreesToRadians(vmult*h_std)));
		}
		if(debug_angles)
			displayAngles();	
	}

	@Override
	public void periodic() {
		//updateOdometry();
	}

	@Override
	public void simulationPeriodic() {
		if(robot_disabled)
			drive(0.0,0.0,0,true); // stay in place
		updateOdometry();
		log();
	}

	void displayAngles(){
		if((cnt%100)==0){
			String str=String.format("angles fl:%-1.2f fr:%-1.2f bl:%-1.2f br:%-1.2f\n",
			m_frontLeft.getAngle(),m_frontRight.getAngle(),m_backLeft.getAngle(),m_backRight.getAngle());
			SmartDashboard.putString("Wheels ", str);
		}
		cnt++;
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

	// A teleop drive method that uses odometry and kinematics
    public void odometryDrive(double xSpeed, double rot) {
		drive(xSpeed, 0, rot, false);
	}
	/** Updates the field relative position of the robot. */
	public void updateOdometry() {
		updatePositions();
		m_poseEstimator.updateWithTime(getClockTime(),m_gyro.getRotation2d(), m_positions);
		//m_poseEstimator.update(m_gyro.getRotation2d(), m_positions);

		// Also apply vision measurements - this must be calculated based either on latency or timestamps.
		if(TargetMgr.tagsPresent()){
			Pose2d vision_pose=TagDetector.getLastPose();
			if(vision_pose !=null){		
				if(use_tags && vision_confidence>0){
					try{
						m_poseEstimator.addVisionMeasurement(vision_pose,getClockTime()-latency);
					}catch(Exception e){
						System.out.println("exception caught in addVisionMeasurement:"+e);
					}
					// TODO: determine actual latency from camera pose 
				}
			}
		}			
		field_pose = getPose();
		log();
	}

	public void updatePositions(){
		m_positions[0]=m_frontLeft.getPosition();
		m_positions[1]=m_frontRight.getPosition();
		m_positions[2]=m_backLeft.getPosition();
		m_positions[3]=m_backRight.getPosition();
	}
	public void resetPositions(){
		m_positions[0]=new SwerveModulePosition();
		m_positions[1]=new SwerveModulePosition();
		m_positions[2]=new SwerveModulePosition();
		m_positions[3]=new SwerveModulePosition();
	}
	public void resetOdometry(Pose2d pose) {
		//reset();
		last_heading = 0;
		m_gyro.reset();
		resetPositions();
		m_kinematics = new SwerveDriveKinematics(
			m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
		m_poseEstimator.resetPosition(gyroRotation2d(), m_positions,pose);

		System.out.println("reset odometry:"+getPose());
		updateOdometry();
	}

	public Pose2d getPose() {
		return m_poseEstimator.getEstimatedPosition();
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
