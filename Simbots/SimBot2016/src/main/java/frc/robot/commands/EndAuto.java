package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class EndAuto extends CommandBase {
    public EndAuto() {
        addRequirements(Robot.simulation);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        System.out.println("EndAuto initialized");
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("EndAuto end");
        Robot.endAuto();
    }
}
