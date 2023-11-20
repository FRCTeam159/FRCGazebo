package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

/**
 *
 */
public class DriveStraight extends TimedCommand{
	static public double P = 0.3;
	static public double I = 0.0;
	static public double D = 0.0;
	static public double TOL = 0.05;
	PIDController pid;
	double distance = 0;
	double tolerance = TOL;
	boolean last_target = false;
	static boolean debug = false;

	public DriveStraight(double t,double d) {
		super(t);
		addRequirements(Robot.driveTrain);
		pid = new PIDController(P, I, D);
		distance = d;
		SmartDashboard.putNumber("P", P);
		SmartDashboard.putNumber("I", I);
		SmartDashboard.putNumber("D", D);
		SmartDashboard.putNumber("TOL", tolerance);
		SmartDashboard.putNumber("DIST", distance);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		super.initialize();
		System.out.println("DriveStraight::initialize()");
		last_target = false;
		double p = SmartDashboard.getNumber("P", P);
		double i = SmartDashboard.getNumber("I", I);
		double d = SmartDashboard.getNumber("D", D);
		tolerance = SmartDashboard.getNumber("TOL", tolerance);
		distance = SmartDashboard.getNumber("DIST", distance);
		pid.setPID(p, i, d);
		//Robot.driveTrain.reset();
		pid.reset();
		pid.setSetpoint(distance);
		pid.setTolerance(tolerance);
		Robot.driveTrain.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		double s = Robot.driveTrain.getDistance();
		double d = pid.calculate(s, distance);
		Robot.driveTrain.tankDrive(d, d);
		if (debug)
			System.out.println("DriveStraight::pidWrite(" + d + ")");
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		boolean new_target = pid.atSetpoint();
		if (new_target && last_target)
			return true;
		last_target = new_target;
		return super.isFinished();
	}

	// Called once after isFinished returns true
	public void end(boolean interrupted){
		System.out.println("DriveStraight::end()");
	}
}
