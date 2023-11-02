package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.ClimberCmds;

import gazebo.SimEncMotor;
import gazebo.SimPiston;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Climber extends SubsystemBase{
	public static double MIN_VALUE=-1;
	public static double MAX_VALUE=1.0;

	SimPiston front_piston;
	SimPiston back_piston;

	
	double front_value=MIN_VALUE;
	double back_value=MIN_VALUE;


	public void initDefaultCommand() {
		 setDefaultCommand(new ClimberCmds());
	}
	
	public Climber() {
		//super();
		front_piston=new SimPiston(4);
		back_piston=new SimPiston(5);
		// front_motor = new SimEncMotor(RobotMap.FRONT_CLIMBER_MOTOR);
		// back_motor = new SimEncMotor(RobotMap.BACK_CLIMBER_MOTOR);
	}
	public void init() {
		enable();
		reset();
	}

	public void reset() {
		setFront(MIN_VALUE);
		setBack(MIN_VALUE);
	}
	public void enable() {
		front_piston.enable();
		back_piston.enable();
	}
	
	public double getFront() {
		return front_value;
	}
	public double getBack() {
		return back_value;
	}
	public void setFront(double v) {
		 front_value=v;
		 front_piston.set(front_value);
	}
	public void setBack(double v) {
		back_value=v;
		back_piston.set(back_value);
	}
	public void log(){
		SmartDashboard.putBoolean("Front Climber", front_value>0?true:false);
		SmartDashboard.putBoolean("Back Climber", back_value>0?true:false);
	}
	@Override
	public void simulationPeriodic() {
		log();
	}
}
