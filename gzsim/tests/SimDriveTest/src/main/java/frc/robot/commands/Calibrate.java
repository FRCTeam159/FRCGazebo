package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import utils.PathData;
import utils.PlotUtils;

public class Calibrate extends CommandBase {
  Timer m_timer;
   private double lastVelocity = 0;
  ArrayList<PathData> plotdata = new ArrayList<PathData>();
 
  private double lastTime = 0;

  int cnt = 0;
  utils.Averager acc_average =new utils.Averager(2);
  utils.Averager vel_average =new utils.Averager(2);
   private static final boolean print = false;

  double max_acc = 0;
  double max_vel = 0;
  double max_pos = 0;

  int plot_type=PlotUtils.PLOT_DYNAMICS;
  double elapsed=0;

  public static int vel_steps=8;
  public static double vel_start=0.5;
  public static double vel_delta=0.1;

  public static double warmup_time = 0.25;
  public static double run_time = 0.5;
  public static double set_value=0;
  double next_step=0;
  double end_time = vel_steps*run_time+warmup_time;
  double last_max_vel=0;
  double max_power=0;

  private final Drivetrain m_drive;

  public Calibrate(Drivetrain drive) {
    SmartDashboard.putNumber("Max Power", max_power);
    SmartDashboard.putNumber("Max Speed", max_vel); 
    m_drive = drive;
    //m_simulation=m_drive.simulation;
    addRequirements(drive);
    m_timer = new Timer();
    m_timer.start();
    m_timer.reset();
  }

  // Called just before this Command runs the first time
  public void initialize() {
    plot_type=PlotUtils.auto_plot_option;
    m_drive.reset(); // reset encoders
    m_timer.start();
    m_timer.reset();
    acc_average.reset();
    vel_average.reset();
    plotdata.clear();
    lastTime = 0;
    lastVelocity = 0;
    m_drive.startAuto();
    cnt = 0;
    set_value=vel_start;
    next_step=warmup_time;
    max_vel=0;
    max_power=0;
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    elapsed = m_drive.getTime();
    if (elapsed > next_step) {
      if (max_vel > 0.1 && (max_vel - last_max_vel) / (max_vel) > 0.05)
        max_power = set_value;
      set_value += vel_delta;
      next_step += run_time;
      last_max_vel = max_vel;
      SmartDashboard.putNumber("Max Power", max_power);
      SmartDashboard.putNumber("Max Speed", max_vel); 
    }
    if (next_step <= end_time) {
      m_drive.set(set_value);
      execute_step();
    }
  }

  private void execute_step(){
    double velocity = m_drive.getVelocity();
    double position = m_drive.getDistance();
    double v = vel_average.getAve(velocity);
    double acceleration = (v - lastVelocity) / 0.02;
    double a = acc_average.getAve(acceleration);
    max_acc = a > max_acc ? a : max_acc;
    max_vel = v > max_vel ? v : max_vel;
    max_pos = set_value;
    double dt = elapsed - lastTime;
    if (print)
      System.out.format("%f %d %f %f %f\n", elapsed, (int) Math.round(dt * 1000.0), position, v,
          a);
    if (PlotUtils.auto_plot_option!=PlotUtils.PLOT_NONE) {
      PathData pd = new PathData();
      pd.tm = position;//elapsed;
      pd.d[0] = set_value;
      pd.d[1] = v;
      if(a<0)
        pd.d[2] = -Math.sqrt(-a);
      else
        pd.d[2] = Math.sqrt(a);
      plotdata.add(pd);
    }

    lastVelocity = v;
    lastTime = elapsed;

  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return elapsed >= end_time;
  }

  @Override
  public void end(boolean interrupted) {
    double a=Math.sqrt(max_acc);
    System.out.println("Calibrate.end()");
    System.out.format("max power=%f max velocity=%f\n", max_power,max_vel); 
    SmartDashboard.putNumber("Max Power", max_power);
    SmartDashboard.putNumber("Max Speed", max_vel); 

    if (PlotUtils.auto_plot_option!=PlotUtils.PLOT_NONE) {
      utils.PlotUtils.publish(plotdata,3,PlotUtils.PLOT_CALIBRATE);
    }
  }
}
