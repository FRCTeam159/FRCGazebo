package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommand extends SequentialCommandGroup {
    public AutonomousCommand() {
		  addCommands(
        new OpenGate(),
        new DrivePath(),
        new StepShooterAngle(5,30),
        new AdjustShot(3),
        new ShootBall(),
        //new OpenGate(2),
        //new DriveStraight(5,4),
        new EndAuto()
      );
    }
}
