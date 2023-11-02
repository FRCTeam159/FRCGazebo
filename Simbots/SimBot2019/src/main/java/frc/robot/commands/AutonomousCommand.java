package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 *
 */
public class AutonomousCommand extends SequentialCommandGroup {
    public AutonomousCommand() {
		//addCommands(new DropGrabber(1.0));
		addCommands(new RaiseElevator(1.0));
	 	//addSequential(new Calibrate());
		addCommands(new ParallelCommandGroup(
			new DropGrabber(1.0),
			new DrivePath(),
			new UntiltElevator(1.0)
		));
		//addCommands(new DriveStraight(8.0));
		//addSequential(new DropGrabber());
		addCommands(new EndAuto());
    }
}
