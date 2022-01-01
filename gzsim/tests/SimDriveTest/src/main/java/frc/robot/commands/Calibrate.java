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
  ArrayList<PathData> plotdata = new ArrayList<PathData>();

  private double lastTime = 0;

  int cnt = 0;
  utils.Averager acc_average =new utils.Averager(20);
  utils.Averager vel_average =new utils.Averager(2);
   private static final boolean print = false;

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
    acc_average.reset();
    vel_average.reset();
    plotdata.clear();
     lastTime = 0;
    lastVelocity = 0;
    m_simulation.reset();
    m_simulation.start();
    m_drive.enable();
    cnt = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    elapsed = m_simulation.getTime();
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
    double v = vel_average.getAve(velocity);
    double acceleration = (v - lastVelocity) / 0.02;
    double a = acc_average.getAve(acceleration);
    max_acc = a > max_acc ? a : max_acc;
    max_vel = v > max_vel ? v : max_vel;
    max_pos = position;
    if (print)
      System.out.format("%f %d %f %f %f\n", elapsed, (int) Math.round(dt * 1000.0), position, v,
          a);
    if (PlotUtils.auto_plot_option!=PlotUtils.PLOT_NONE) {
      PathData pd = new PathData();
      pd.tm = elapsed;
      pd.d[0] = position;
      pd.d[1] = v;
      pd.d[2] = a;
      plotdata.add(pd);
    }

    lastVelocity = v;
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

    if (PlotUtils.auto_plot_option!=PlotUtils.PLOT_NONE) {
      String label_list[]={"","",""};
    
      label_list[0]=String.format("Position  %1.2f",max_pos);
      label_list[1]=String.format("Velocity  %1.2f",max_vel);
      label_list[2]=String.format("Accel     %1.2f",max_acc);
      PlotUtils.genericTimePlot(plotdata,label_list,3);
      //PlotUtils.plotCalibration(plotdata,max_pos,max_vel,max_acc);
    }
  }

}
