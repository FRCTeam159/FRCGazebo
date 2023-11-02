package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;

/**
 *
 */
public class ClimberCmds extends CommandBase implements RobotMap{
	public static double MIN_VALUE=-2;
	public static double MAX_VALUE=2.0;
	
	static double front_value=0;
	static double back_value=0;


	public ClimberCmds() {
		// Use addRequirements() here to declare subsystem dependencies
		addRequirements(Robot.climber);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("ClimberCmds.initialize()");
		front_value=MIN_VALUE;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		XboxController stick = Robot.controller;
		front_value=Robot.climber.getFront();
		back_value=Robot.climber.getBack();

		// dpad
		int pov= stick.getPOV();
		if(pov>=0){
		  switch(pov){
			case 0: 
			  front_value=MAX_VALUE;
			  break;
			case 90:
			  back_value=MAX_VALUE;
			  break;
			case 180:
			  front_value=MIN_VALUE;
			  break;
			case 270:
			  back_value=MIN_VALUE;
			  break;
		  }    
		}
		
		Robot.climber.setFront(front_value);
		Robot.climber.setBack(back_value);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted){
		System.out.println("ClimberCmds::end()");
	}
}
