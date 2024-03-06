// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gazebo.SimEncMotor;

public class SwerveModule {
  private static final double kWheelRadius = Units.inchesToMeters(2.0); // 2 inches

  private double distancePerRotation=2*kWheelRadius*Math.PI;

  
  private SimEncMotor m_driveMotor;
	private SimEncMotor m_turnMotor;

  public static String chnlnames[] = { "BR", "BL", "FL", "FR" };

  String name;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(10, 0.0, 0);
  private final PIDController m_turningPIDController = new PIDController(8, 0.0, 0);
  // private final ProfiledPIDController m_turningPIDController =
  //     new ProfiledPIDController(5,0.0,0,
  //         new TrapezoidProfile.Constraints(
  //             kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.3);

  public int m_drive_chnl;
  public int m_turn_chnl;
  public int chnl;
  
  boolean m_enabled=false;

  static boolean debug_states=true;
  static boolean debug_align=false;
  static boolean debug_correction=false;

  static public boolean optimize=true;
  boolean aligning=false;
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   * @param i
   *
   */
  public SwerveModule(int driveMotorChannel,int turningMotorChannel, int i) {
    m_drive_chnl=driveMotorChannel;
    m_turn_chnl=turningMotorChannel;

    m_driveMotor=new SimEncMotor(driveMotorChannel);
    m_turnMotor=new SimEncMotor(turningMotorChannel);

    m_driveMotor.setDistancePerRotation(distancePerRotation);
		m_turnMotor.setDistancePerRotation(2*Math.PI); // getDistance will return degrees*rotations

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.setTolerance(Math.toRadians(2));
    chnl=i-1;
    name = chnlnames[chnl];
    if(debug_states)
      SmartDashboard.putString(name, "Working");
  }

  public void enable(){
    m_enabled=true;
    m_driveMotor.enable();
    m_turnMotor.enable();
  }
  public void disable(){
    m_enabled=false;
    m_driveMotor.disable();
    m_turnMotor.disable();
  }
  public void reset(){
    m_drivePIDController.reset();
    m_turningPIDController.reset();
    m_driveMotor.reset();
    m_turnMotor.reset();
  }

  public double heading(){
    return m_turnMotor.getDistance();
  }
  
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(heading());
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getRate(),  getRotation2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getDistance(),  getRotation2d());
  }

  public double getDistance(){
    return m_driveMotor.getDistance();
  }

  public double getVelocity() {
      return m_driveMotor.getRate();
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    //SwerveModuleState state = desiredState;  // don't optimize
    SwerveModuleState state=desiredState;
    if(optimize){
      SwerveModuleState newstate =SwerveModuleState.optimize(desiredState, getRotation2d());
      state=newstate;
    }
  
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getRate(), state.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    
    double turn_angle=getRotation2d().getRadians(); // rotations in radians

    final double turnOutput = m_turningPIDController.calculate(turn_angle,state.angle.getRadians());
    final double turnFeedforward = m_turnFeedforward.calculate(state.angle.getRadians());

    double set_drive=driveOutput+driveFeedforward;
    double set_turn=turnOutput+turnFeedforward;
    if(debug_states){
      String s;
      if(debug_correction)
        s = String.format("DC %-2.2f RC %-3.1f\n", set_drive,set_turn); 
      else
        s = String.format("Pos %-2.2f Rot %-3.1f\n", getDistance(),Math.toDegrees(heading())%360); 
      
      SmartDashboard.putString(name, s);
    }
    m_driveMotor.set(set_drive);
    m_turnMotor.set(set_turn);
  }
  public void resetWheel(boolean begin){
    aligning=begin;
    alignWheel();
  }
  public void alignWheel(){
     setAngle(0,0);
  }
  public boolean wheelReset() {
    return m_turningPIDController.atSetpoint();
  }
  // just apply a voltage to the turn motor
  public void turn(double value){
    m_turnMotor.set(value);
  }

  // just apply a voltage to the wheel motor
  public void move(double value){
    m_driveMotor.set(value);
  }
  // use a PID controller to set an explicit turn angle
  public void setAngle(double a, double d){
    double r=Math.toRadians(a);
    m_turningPIDController.setSetpoint(r);
    double current=getRotation2d().getRadians(); // rotations in radians
    double turnOutput =m_turningPIDController.calculate(current,r);
    m_driveMotor.set(d);
    m_turnMotor.set(turnOutput); 
    if(debug_align && aligning && chnl==0){
       String s = String.format("%s Target:%-2.2f Current:%-3.3f Correction:%-3.3f", 
      name,a,Math.toDegrees(heading())%360,turnOutput); 
      System.out.println(s);
    }
  }
  
  public double getAngle(){
    return  m_turnMotor.getDistance();
  }
  public double getMoveRate(){
    return  m_driveMotor.getRate();
  }
  public double getTurnRate(){
    return  m_turnMotor.getRate();
  }
  public void setInverted(){
    m_driveMotor.setInverted();
  }
}
