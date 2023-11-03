
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.objects.PlotServer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Simulation;
import frc.robot.subsystems.VisionProcess;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

	public static  DriveTrain driveTrain= new DriveTrain();
	public static  Climber climber = new Climber();
	public static  Elevator elevator= new Elevator();
	public static  Grabber grabber= new Grabber();
	public static  VisionProcess vision = new VisionProcess();
	public static  XboxController controller = new XboxController(0);
	public static  Simulation simulation = new Simulation();
	public static  Autonomous autonomous = new Autonomous();

	public static boolean isAuto = false;
	public static boolean isTele = false;
	public static boolean doAuto = true;
	public static boolean haveAuto = true;
	public static boolean hatchMode = true;
	public static boolean cargoMode = true;

	PlotServer plotsub=new PlotServer();

	public static double auto_scale = 0.75;
	// SendableChooser<Command> chooser = new SendableChooser<>();
	Command autonomousCommand= null;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		driveTrain.initDefaultCommand();
		elevator.initDefaultCommand();
		grabber.initDefaultCommand();
		climber.initDefaultCommand();

		System.out.println("robotInit");
		simulation.init();
		
		vision.init();
        vision.start();
		plotsub.start();
		reset();
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
		//driveTrain.resetGyro();
		reset();
		System.out.println("disabledInit");
	}

	@Override
	public void disabledPeriodic() {
		//oi.buttonTest();
		//Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		// autonomousCommand = chooser.getSelected();
		driveTrain.resetGyro();
		System.out.println("autonomousInit");
		if (doAuto) {
			isAuto = true;
			isTele = false;
		} else {
			isAuto = false;
			isTele = true;
		}
		CommandScheduler.getInstance().cancelAll();
		autonomousCommand =autonomous.getCommand();
		// schedule the autonomous command (example)
		if (doAuto && autonomousCommand != null)
			autonomousCommand.schedule();
	}

	@Override
	public void teleopInit() {
		simulation.endAuto();
		System.out.println("teleopInit");
		driveTrain.resetGyro();
		isAuto = false;
		isTele = true;
		if (doAuto && autonomousCommand != null)
			autonomousCommand.cancel();
	}

	public static void reset(){
		simulation.endAuto();
		driveTrain.reset();
		climber.reset();
		grabber.init();
		elevator.init();
		climber.init();
	}
	
}
