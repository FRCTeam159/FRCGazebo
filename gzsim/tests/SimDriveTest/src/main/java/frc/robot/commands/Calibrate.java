package frc.robot.commands;

import java.util.ArrayList;
import java.util.LinkedList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import utils.PathData;
import utils.PlotUtils;
import frc.robot.subsystems.Simulation;

/**
 *
 */
public class Calibrate extends CommandBase {
  Timer m_timer;
  double runtime = 4;
  private double lastVelocity = 0;

  private LinkedList<Double> vals = null;
  private double total = 0;
  private double lastTime = 0;
  int averages = 5;
  int cnt = 0;
  ArrayList<PathData> plotdata = new ArrayList<PathData>();
  private static final boolean plot = true;
  private static final boolean print = true;

  double max_acc = 0;
  double max_vel = 0;
  double max_pos = 0;

  double elapsed=0;

  private final Drivetrain m_drive;
  private final Simulation m_simulation;

  public Calibrate(Drivetrain drive) {
    m_drive = drive;
    m_simulation=m_drive.simulation;
    addRequirements(drive);
    m_timer = new Timer();
    m_timer.start();
    m_timer.reset();
  }

  // Called just before this Command runs the first time
  public void initialize() {
    m_drive.reset(); // reset encoders
    m_timer.start();
    m_timer.reset();
    vals = new LinkedList<Double>();
    lastTime = 0;
    lastVelocity = 0;
    m_simulation.reset();
    m_simulation.start();
    m_drive.enable();
    cnt = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    elapsed = m_timer.get();
    //elapsed=m_simulation.getTime();
    if (cnt < 1) {
      lastTime = elapsed;
      cnt++;
      return;
    }
    double dt = elapsed - lastTime;

    m_drive.set(10.0);

    double velocity = m_drive.getVelocity();
    double position = m_drive.getDistance();
    double acceleration = (velocity - lastVelocity) / dt;
    double aveAcceleration = rollingAverage(acceleration, averages);
    max_acc = aveAcceleration > max_acc ? aveAcceleration : max_acc;
    max_vel = velocity > max_vel ? velocity : max_vel;
    max_pos = position;
    if (print)
      System.out.format("%f %d %f %f %f\n", elapsed, (int) Math.round(dt * 1000.0), position, velocity,
          aveAcceleration);
    if (plot) {
      PathData pd = new PathData();
      pd.tm = elapsed;
      pd.d[0] = position;
      pd.d[1] = velocity;
      pd.d[2] = aveAcceleration;
      plotdata.add(pd);
    }

    lastVelocity = velocity;
    lastTime = elapsed;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return elapsed >= runtime;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Calibrate.end()");
    System.out.format("max vel=%f max acc=%f\n", max_vel, max_acc);
    m_simulation.end();

    if (plot) {
      PlotUtils.plotCalibration(plotdata,max_pos,max_vel,max_acc);
    }
  }

  private double rollingAverage(double d, int aves) {
    if (vals.size() == aves)
      total -= vals.removeFirst().doubleValue();
    vals.addLast(d);
    total += d;
    return total / vals.size();
  }

}
