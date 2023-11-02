package frc.robot.commands;

import java.util.ArrayList;
import java.util.LinkedList;
import frc.robot.PathData;
import frc.robot.subsystems.PlotPath;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class Calibrate extends CommandBase {
  //Timer mytimer;
  double runtime = 5;
  private double lastVelocity = 0;

  private LinkedList<Double> vals = null;
  private double total = 0;
  private double lastTime = 0;
  int averages = 2;
  int cnt = 0;
  ArrayList<PathData> plotdata = new ArrayList<PathData>();
  private static final boolean plot = true;
  private static final boolean print = true;

  double max_acc = 0;
  double max_vel = 0;
  long start_time;
  double TIME_STEP = 0.02;
	private int pathIndex = 0;


  public Calibrate() {
    addRequirements(Robot.driveTrain);
   //mytimer = new Timer();
   // mytimer.start();
   // mytimer.reset();
  }

  static public double f2m(double x) {
    return x * 12 * 0.0254;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Robot.driveTrain.reset();
    //mytimer.start();
    //mytimer.reset();
    System.out.println("Calibrate.initialize(" + Robot.auto_scale + ")");
    vals = new LinkedList<Double>();
    lastTime = 0;
    lastVelocity = 0;
    Robot.driveTrain.enable();
    start_time = System.nanoTime();
    cnt = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double curtime = getSimTime();
    if (cnt < 1) {
      lastTime = curtime;
      cnt++;
      pathIndex++;
      return;
    }
    double dt = curtime - lastTime;
    if (curtime < 0.5)
      Robot.driveTrain.set(0, 0);
    else
      Robot.driveTrain.set(Robot.auto_scale, Robot.auto_scale);

    double velocity = Robot.driveTrain.getVelocity();
    double position = Robot.driveTrain.getDistance();
    double acceleration = (velocity - lastVelocity) / dt;
    double aveAcceleration = rollingAverage(acceleration, averages);
    max_acc = aveAcceleration > max_acc ? aveAcceleration : max_acc;
    max_vel = velocity > max_vel ? velocity : max_vel;

    if (print)
      System.out.format("%f %f %f %f\n", curtime, f2m(position), f2m(velocity),
          f2m(aveAcceleration));
    if (plot) {
      PathData pd = new PathData();
      pd.tm = curtime;
      pd.d[0] = f2m(position);
      pd.d[1] = f2m(velocity);
      pd.d[2] = f2m(aveAcceleration);
      plotdata.add(pd);
    }

    lastVelocity = velocity;
    lastTime = curtime;
    pathIndex++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (getSimTime() >= runtime)
      return true;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("Calibrate.end()");
    Robot.driveTrain.disable();
    System.out.format("max vel=%f max acc=%f\n", f2m(max_vel), f2m(max_acc));

    if (plot) {
      new PlotPath(plotdata, 3);
    }
  }

  private double rollingAverage(double d, int aves) {
    if (vals.size() == aves)
      total -= vals.removeFirst().doubleValue();
    vals.addLast(d);
    total += d;
    return total / vals.size();
  }

  double getSimTime() {
		return (double) TIME_STEP * pathIndex;
	}
}
