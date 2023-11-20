package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class TurnToAngle extends CommandBase{
    double angle;
    PIDController pid;    
    Timer m_timer=new Timer();
    double timeout;

    public TurnToAngle(double angle,double tm) {
      addRequirements(Robot.driveTrain);
      this.angle=angle;
      timeout=tm;
      pid = new PIDController(0.02, 0.0, 0.0);
      m_timer.start();
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
      pid.reset();
      pid.setSetpoint(angle);
      pid.setTolerance(1.0,0.5);
      m_timer.reset();
      System.out.println("TurnToAngle::initialize:"+angle);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      double h=Robot.driveTrain.getHeading();
      double d=pid.calculate(h,angle);
      Robot.driveTrain.set(-d, d);
      //System.out.println("h:"+h+" d:"+d);
    }
    @Override
    public boolean isFinished() {
      if(m_timer.get()>timeout || pid.atSetpoint())
        return true;
      return false;
    }
    // Called once after timeout
    @Override
    public void end(boolean interrupted) {
      if(m_timer.get()>timeout)
        System.out.println("TurnToAngle.end(timeout exceeded)");
      else if( pid.atSetpoint())
        System.out.println("TurnToAngle.end(at target)");
      else if(interrupted)
        System.out.println("TurnToAngle.end(interupted)");
      else
        System.out.println("TurnToAngle.end(??)");
    }
}
