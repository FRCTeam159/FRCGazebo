package frc.robot.commands;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetElevator extends CommandBase {
    double pos=0;
    Timer m_timer=new Timer();
    double timeout;

    public SetElevator(double value, double tm) {
        pos=value;
        timeout=tm;
        m_timer.start();
        m_timer.reset();  
        addRequirements(Robot.elevator);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
      System.out.println("SetElevator.initialize "+pos);
      Robot.elevator.enable();
      Robot.driveTrain.enable();
      Robot.elevator.setPosition(pos);

      m_timer.start();
      m_timer.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      //Robot.elevator.setPosition(pos);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public boolean isFinished() {
      if(m_timer.get()>timeout || Robot.elevator.atTarget())
        return true;
      return false;
    }

    // Called once after timeout
  @Override
  public void end(boolean interrupted) {
    if(m_timer.get()>timeout)
      System.out.println("SetElevator.end(timeout exceeded)");
    else if(Robot.elevator.atTarget())
      System.out.println("SetElevator.end(at target)");
    else if(interrupted)
       System.out.println("SetElevator.end(interupted)");
    else
      System.out.println("SetElevator.end(??)");
  }
}
