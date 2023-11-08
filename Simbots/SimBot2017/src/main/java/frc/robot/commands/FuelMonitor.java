package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.XboxController;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class FuelMonitor extends CommandBase implements RobotMap{
    static final double FORWARDVOLTAGE=1;
    static final double REVERSEVOLTAGE=-0.5;
    static final int WAITFORBUTTON=1;
	static final int WAITFORUPPERLIMIT=2;
	static final int WAITFORLOWERLIMIT=3;
	int state = WAITFORBUTTON;

    XboxController stick = Robot.controller;

    public FuelMonitor() {
        addRequirements(Robot.fuelSubsystem);
    }
    @Override
    public void initialize() {
        System.out.println("FuelMonitor started");
        state = WAITFORBUTTON;
    }

    @Override
    public void execute() {
        switch(state){
        default:
        case WAITFORBUTTON:
            if(Robot.controller.getRawButton(FUELPUSHERBUTTON))
                state=WAITFORUPPERLIMIT;
            else
                Robot.fuelSubsystem.set(0);
            break;
        case WAITFORUPPERLIMIT:
            if(Robot.fuelSubsystem.atUpperLimit())
                state= WAITFORLOWERLIMIT;
            else
                Robot.fuelSubsystem.set(FORWARDVOLTAGE);
            break;
        case WAITFORLOWERLIMIT:
            if(Robot.fuelSubsystem.atLowerLimit())
                state= WAITFORBUTTON;
            else
                Robot.fuelSubsystem.set(REVERSEVOLTAGE);
            break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("FuelMonitor.end");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
