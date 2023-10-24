package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraight extends CommandBase{
  Timer mytimer;

  static public double P = 0.5;
  static public double I = 0.0;
  static public double D = 0.0;
  static public double TOL = 0.05;
  PIDController pid;
  double distance = 0;
  double tolerance = TOL;
  boolean last_target = false;
  //PIDSourceType type = PIDSourceType.kDisplacement;
  static boolean debug = false;
  double last_time;
  int count = 0;
  double speed=1;
  double heading;
  double tm;
  boolean useGyro=false;

  public DriveStraight(double d, double s, double t) {
    this(d, s, t, 0.0);
    useGyro=false;
  }
  public DriveStraight(double d, double s, double t,double h) {
    addRequirements(Robot.driveTrain);
    pid = new PIDController(P, I, D);
    tm=t;
    distance = d;
    speed=s;
    heading=h;
    mytimer = new Timer();
    mytimer.start();
    mytimer.reset();
    count = 0;
    if (debug) {
      SmartDashboard.putNumber("P", P);
      SmartDashboard.putNumber("I", I);
      SmartDashboard.putNumber("D", D);
      SmartDashboard.putNumber("TOL", tolerance);
      SmartDashboard.putNumber("DIST", distance);
    }
    useGyro=Robot.useGyro;
  }

  // Called just before this Command runs the first time
  public void initialize() {
    System.out.println("DriveStraight::initialize()");
    last_target = false;
    mytimer.start();
    mytimer.reset();
    if (debug) {
      double p = SmartDashboard.getNumber("P", P);
      double i = SmartDashboard.getNumber("I", I);
      double d = SmartDashboard.getNumber("D", D);
      tolerance = SmartDashboard.getNumber("TOL", tolerance);
      distance = SmartDashboard.getNumber("DIST", distance);
      pid.setPID(p, i, d);
    }
    Robot.driveTrain.resetEncoders();
    pid.reset();
    pid.setSetpoint(distance);
    pid.setTolerance(tolerance);
    //pid.enable();
    Robot.driveTrain.enable();
    last_time = mytimer.get();
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    double s = 12*Robot.driveTrain.getDistance();
    double d = pid.calculate(s, distance);
    double gh = Robot.driveTrain.getHeading();
    double herr=heading-gh;
    double turn=0;
    if (useGyro)
      turn = Robot.GFACT * (-1.0 / 180.0) * herr;

    double lval = d + turn;
    double rval = d - turn;

    lval=clamp(lval);
    rval=clamp(rval);
    if (debug)
      System.out.println("DriveStraight::pidWrite(" + d + ","+turn+")");
    Robot.driveTrain.set(lval, rval);
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    if(super.isFinished())
      return true;
    boolean new_target = pid.atSetpoint();
    if (new_target && last_target)
      return true;
    last_target = new_target;
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    System.out.println("DriveStraight::end()");
   // pid.disable();
  }
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    System.out.println("DriveStraight::interrupted()");
    end();
  }

  double clamp(double d) {
    if(d>speed)
      d=speed;
    else if(d<-speed)
      d=-speed;
    return d;
  }

}
