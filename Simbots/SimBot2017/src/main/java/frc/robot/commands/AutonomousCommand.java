package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 *
 */
public class AutonomousCommand extends SequentialCommandGroup {
    public AutonomousCommand() {
		addCommands(new DrivePath());
    }
}
