package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedCommand extends CommandBase {
    Timer m_timer;
    double timeout;
    public TimedCommand(double t) {
        m_timer = new Timer();
        m_timer.start();
        m_timer.reset();
        timeout=t;
    }
    @Override
    public void initialize() {
        m_timer.reset();
    }

    @Override
     public boolean isFinished() {
         return m_timer.get()>=timeout;
     }
}
