package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommand extends SequentialCommandGroup {
    public AutonomousCommand() {
		  addCommands(
        new DrivePath(),
        //new OpenGate(2),
        //new DriveStraight(5,4),
        new EndAuto()
      );
    }
}
