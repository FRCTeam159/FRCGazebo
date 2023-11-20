package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class SetGrabberState extends CommandBase implements Constants {
    int state=HOLD;
    Timer m_timer=new Timer();
    double timeout;
    public SetGrabberState(int state, double tm) {
      this.state=state;
      m_timer.start();
      m_timer.reset();
      timeout=tm;
      addRequirements(Robot.cubeHandler);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
      System.out.println("SetGrabberState.initialize:"+state);
      m_timer.start();
      m_timer.reset();
      Robot.cubeHandler.setState(state);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      Robot.cubeHandler.setState(state);
    }

    // Called once after timeout
    @Override
    public void end(boolean interrupted) {
      System.out.println("SetGrabberState.end");
      Robot.cubeHandler.hold();
    }
    @Override
    public boolean isFinished() {
      if(m_timer.get()>timeout)
        return true;
      return false;
    }
   
}
