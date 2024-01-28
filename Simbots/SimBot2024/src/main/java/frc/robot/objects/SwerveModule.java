// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import gazebo.SimEncMotor;

public class SwerveModule {
  private static final double kWheelRadius = Units.inchesToMeters(2.0); // 2 inches

  private double gazebo_scale=1;  // not sure why but grid marks in gazebo are somewhat larger than a meter ?
  private double distancePerRotation=gazebo_scale*2*kWheelRadius*Math.PI;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed; //degrees per second;
  private static final double kModuleMaxAngularAcceleration = Drivetrain.kMaxAngularAcceleration;// degrees per second per second

  private SimEncMotor m_driveMotor;
	private SimEncMotor m_turnMotor;

  public static String chnlnames[] = { "BR", "BL", "FL", "FR" };

  String name;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(10, 0.0, 0);
  //private final PIDController m_turningPIDController = new PIDController(3, 0.05, 0);
  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(5,0.0,0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.1);

  public int m_drive_chnl;
  public int m_turn_chnl;
  
  boolean m_enabled=false;

  static boolean debug=false;
  static public boolean optimize=true;
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

    name = chnlnames[i - 1];
    if(debug)
      SmartDashboard.putString(name, "Working");
  }

  public void enable(){
    m_enabled=true;
    m_driveMotor.enable();
    m_turnMotor.enable();
  }
  public void disable(){
   // m_turningPIDController.reset(0.0);

    m_enabled=false;
    m_driveMotor.disable();
    m_turnMotor.disable();
  }
  public void reset(){
    //m_turningPIDController.reset(0.0);
    m_drivePIDController.reset();
    m_turningPIDController.reset(0.0);
    m_enabled=false;
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
      //if(newstate.angle.getDegrees()!=state.angle.getDegrees())
     //   System.out.println(name+"-optimized "+state.angle.getDegrees()+":"+newstate.angle.getDegrees());
      state=newstate;
    }
   
    double velocity=getVelocity();
    
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getRate(), state.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    
    double turn_angle=getRotation2d().getRadians(); // rotations in radians

    final double turnOutput = m_turningPIDController.calculate(turn_angle,state.angle.getRadians());
    final double turnFeedforward = 0;//m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    //final double turnFeedforward = m_turnFeedforward.calculate(state.angle.getRadians());

    double set_drive=driveOutput+driveFeedforward;
    double set_turn=turnOutput+turnFeedforward;

    if(debug){
      String s = String.format("Vel %-2.2f(%-2.2f) -> %-2.2f Angle %-3.3f(%-2.3f) -> %-2.3f\n", 
      velocity,state.speedMetersPerSecond,set_drive,Math.toDegrees(turn_angle), state.angle.getDegrees(), set_turn); 
      SmartDashboard.putString(name, s);
    }
      m_driveMotor.set(set_drive);
      m_turnMotor.set(set_turn);
  }
  public void resetWheel(){
    m_turnMotor.setPosition(0);
    m_turnMotor.setPosition();
  }
  public boolean wheelReset(){
    return m_turnMotor.atTarget();
  }
  // just apply a voltage to the turn motor
  public void turn(double value){
    m_turnMotor.set(value);
  }

  public double getXVoltage(){
    return  m_driveMotor.get();
  }
  public double getYVoltage(){
    return  m_turnMotor.get();
  }
  // just apply a voltage to the wheel motor
  public void move(double value){
    m_driveMotor.set(value);
  }
  // use a PID controller to set an explicit turn angle
  public void setAngle(double a, double d){
    double current=m_turnMotor.getDistance();
    current=current%360;
    double turnOutput = m_turningPIDController.calculate(Math.toRadians(current),Math.toRadians(a) );
    //System.out.println(m_drive_chnl/2+" "+a+" "+current+" "+turnOutput+" "+d);
    m_driveMotor.set(d);
    m_turnMotor.set(turnOutput); 
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
