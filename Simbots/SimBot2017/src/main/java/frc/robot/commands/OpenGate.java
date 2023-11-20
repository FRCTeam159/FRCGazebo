package frc.robot.commands;
import frc.robot.Robot;

public class OpenGate extends TimedCommand {
    OpenGate(double t){
        super(t);
        addRequirements(Robot.gearSubsystem);
    }
    @Override
    public void initialize() {
        System.out.println("OpenGate started");
        Robot.gearSubsystem.Open();
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("OpenGate.end");
    }
    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
