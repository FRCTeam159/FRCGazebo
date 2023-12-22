
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Random;

import objects.PlotServer;
import frc.robot.subsystems.AutoSelector;
import frc.robot.subsystems.CubeHandler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import subsystems.Simulation;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot implements RobotMap, PhysicalConstants, Constants {

  public static DriveTrain driveTrain;
  public static Elevator elevator;
  public static CubeHandler cubeHandler;
  public static AutoSelector autoSelector;
  public static Simulation simulation;
  public static XboxController controller;
  
  static SendableChooser<Integer> position_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> plot_chooser = new SendableChooser<Integer>();

  Command autonomousCommand;
  public static double auto_scale = 0.8;
  public static boolean calibrate = false;
  public static boolean test = true;

  public static boolean useGyro = true;

  // public static double MAX_VEL = 1.7;
  // public static double MAX_ACC = 2.6;
  // public static double MAX_JRK = 1.3;

  public static double MAX_VEL = 0.75;
  public static double MAX_ACC = 0.75;
  public static double MAX_JRK = 0.25;

  public static double KP = 3.0;
  public static double KD = 0.0;
  public static double GFACT = 10.0;

  Random random = new Random();

  public static Integer robotPosition = POSITION_RIGHT;
  public static Integer fms_pattern = robotPosition;
  public static String fms_string = "RRR";

  PlotServer m_plotsub=new PlotServer();


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    String pos = System.getenv("POSITION");
    driveTrain = new DriveTrain();
    elevator = new Elevator();
    cubeHandler = new CubeHandler();
    autoSelector = new AutoSelector();
    simulation = new Simulation();
    controller = new XboxController(0);
    
    driveTrain.initDefaultCommand();
    elevator.initDefaultCommand();
    cubeHandler.initDefaultCommand();

    try {
      robotPosition = Integer.parseInt(pos);
    } catch (NumberFormatException e) {
      robotPosition = POSITION_CENTER;
    }
       //ITable table=position_chooser.getTable() // returns null until disabledInit called
    position_chooser.addOption("Right", POSITION_RIGHT);
    position_chooser.addOption("Left", POSITION_LEFT);
    position_chooser.addOption("Center", POSITION_CENTER);

    switch (robotPosition) { 
    case POSITION_CENTER:
      position_chooser.setDefaultOption("Center", POSITION_CENTER);
      System.out.println("CENTER");
      break;
    case POSITION_RIGHT:
      position_chooser.setDefaultOption("Right", POSITION_RIGHT);
      System.out.println("RIGHT");
      break;
    case POSITION_LEFT:
      position_chooser.setDefaultOption("Left", POSITION_LEFT);
      System.out.println("LEFT");
      break;
    }
    
    plot_chooser.setDefaultOption("None", utils.PlotUtils.PLOT_NONE);
    plot_chooser.addOption("Position", utils.PlotUtils.PLOT_POSITION);
    plot_chooser.addOption("Dynamics", utils.PlotUtils.PLOT_DYNAMICS);

    SmartDashboard.putBoolean("UseGyro", useGyro);

    SmartDashboard.putNumber("MAX_VEL", MAX_VEL);
    SmartDashboard.putNumber("MAX_ACC", MAX_ACC);
    SmartDashboard.putNumber("MAX_JRK", MAX_JRK);
    SmartDashboard.putNumber("KP", KP);
    SmartDashboard.putNumber("KD", KD);
    SmartDashboard.putNumber("GFACT", GFACT);
    SmartDashboard.putString("FMS-STR", "RRR");

    SmartDashboard.putString("Target", "Calculating");
    SmartDashboard.putBoolean("Test", true);

    SmartDashboard.putNumber("Auto Scale", auto_scale);
    SmartDashboard.putData("Position", position_chooser);
    SmartDashboard.putData("Plot", plot_chooser);
    simulation.init();
    driveTrain.init();
    m_plotsub.start();
    //robotPosition = position_chooser.getSelected(); // ITable not valid here ?? 
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }
  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    simulation.endAuto();
    getDataFromDashboard();
    System.out.println("disabledInit");
    CommandScheduler.getInstance().cancelAll();
    driveTrain.reset();
    driveTrain.disable();
    simulation.disable();
    if (autonomousCommand != null)
      autonomousCommand.cancel();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    System.out.println("autonomousInit");
    getDataFromDashboard();
    reset();
    CommandScheduler.getInstance().cancelAll();
    //if (autonomousCommand != null)
    //  autonomousCommand.cancel();

    setFMS();
    autonomousCommand = autoSelector.getAutonomous();
    // schedule the autonomous command (example)
    if (autonomousCommand != null)
      autonomousCommand.schedule();
      simulation.startAuto();
  }


  @Override
  public void teleopInit() {
    simulation.endAuto();
    getDataFromDashboard();
    System.out.println("teleopInit");
    reset();
    //CommandScheduler.getInstance().cancelAll();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null)
      autonomousCommand.cancel();
  }

  
  void reset() {
    driveTrain.reset();
    elevator.reset();
    cubeHandler.reset();
    cubeHandler.enable();
    elevator.enable();
    driveTrain.enable();
  }

  void getDataFromDashboard() {
    robotPosition = position_chooser.getSelected();
    useGyro = SmartDashboard.getBoolean("UseGyro", useGyro);
    MAX_VEL = SmartDashboard.getNumber("MAX_VEL", MAX_VEL);
    MAX_ACC = SmartDashboard.getNumber("MAX_ACC", MAX_ACC);
    MAX_JRK = SmartDashboard.getNumber("MAX_JRK", MAX_JRK);
    GFACT = SmartDashboard.getNumber("GFACT", GFACT);
    KP = SmartDashboard.getNumber("KP", KP);
    KD = SmartDashboard.getNumber("KD", KD);
    auto_scale = SmartDashboard.getNumber("Auto Scale", auto_scale);
  }

  void setFMS() {
    test = SmartDashboard.getBoolean("Test", false);
    if (test) {
      fms_string = SmartDashboard.getString("FMS-STR", "RRR");
    } else {
      String fms[] = { "LLL", "LLR", "LRL", "LRR", "RLL", "RLR", "RRL", "RRR" };
      double rand = random.nextDouble();

      fms_pattern = (int) (7.99999 * rand); // 0-7
      fms_string = fms[fms_pattern];
      System.out.println("rand=" + rand + " indx=" + fms_pattern);
      SmartDashboard.putString("FMS-STR", fms[fms_pattern]);
    }
  }

  public static int getPlotOption() {
    return plot_chooser.getSelected();
  }
}
