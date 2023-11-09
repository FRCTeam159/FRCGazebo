package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommand extends SequentialCommandGroup {
    public AutonomousCommand() {
		  addCommands(
        new DrivePath(),
        new OpenGate(2),
        new DriveStraight(4,-1),
        new EndAuto()
      );
    }
}
