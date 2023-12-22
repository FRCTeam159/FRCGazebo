
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import objects.PlotServer;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Holder;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import subsystems.Simulation;
import frc.robot.subsystems.VisionProcess;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot implements RobotMap {
	public static  XboxController controller = new XboxController(0);
	public static  DriveTrain driveTrain= new DriveTrain();
	
	public static  Simulation simulation = new Simulation();
	public static  Autonomous autonomous = new Autonomous();
	public static  VisionProcess vision = new VisionProcess();
	public static  Loader loader = new Loader();
	public static  Holder holder = new Holder();
	public static  Shooter shooter = new Shooter();

	PlotServer plotsub=new PlotServer();

	Command autonomousCommand= null;

	public static enum Mode {
		INIT,
		SHOOTING,
		LOADING
	}

	public static Mode mode=Mode.SHOOTING;
	boolean ball_was_present=true;
	public static boolean in_auto=false;
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		driveTrain.initDefaultCommand();
		System.out.println("robotInit");
		simulation.init();
		plotsub.start();
		vision.init();
        vision.start();
		loader.initDefaultCommand();
		holder.initDefaultCommand();
		shooter.initDefaultCommand();
		shooter.init();
		loader.init();
		reset();
	}
	@Override
    public void robotPeriodic() {
		boolean m = holder.isBallPresent();

		switch(mode){
			case INIT:
			if(m)
				mode = Mode.SHOOTING;
			break;
			case SHOOTING:
			//if(!m && ball_was_present)
			if(!m)
				mode = Mode.LOADING;
			break;
			case LOADING:
			//if(m && !ball_was_present)
			if(m)
				mode = Mode.SHOOTING;
			break;
		}
		ball_was_present = m;
		

		CommandScheduler.getInstance().run();
		log();
	}

  @Override
	public void teleopPeriodic() {
		
	}
	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

		// if (autonomousCommand != null)
		// 	autonomousCommand.cancel();
		// autonomousCommand=null;
		//CommandScheduler.getInstance().cancelAll();
		//reset();
		in_auto=false;

		System.out.println("disabledInit");
	}

	@Override
	public void disabledPeriodic() {
	
	}

	@Override
	public void autonomousInit() {
		driveTrain.resetGyro();
		System.out.println("autonomousInit");
		in_auto=true;

		
		//CommandScheduler.getInstance().cancelAll();
		autonomousCommand =autonomous.getCommand();
		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.schedule();
	}

	@Override
	public void teleopInit() {
		simulation.endAuto();
		in_auto=false;

		System.out.println("teleopInit");
		//driveTrain.resetGyro();

		//CommandScheduler.getInstance().cancelAll();
		
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		autonomousCommand=null;

		reset();

	}

	public static void reset(){
		simulation.endAuto();
		driveTrain.reset();
		shooter.reset();
		loader.reset();
		holder.reset();
		mode=Mode.SHOOTING;
		in_auto=false;
	}

	void log(){
		SmartDashboard.putBoolean("Shooting Mode", mode==Robot.Mode.SHOOTING);

		holder.log();
		shooter.log();
		loader.log();
	}

	public static double getTime(){
		return simulation.getSimTime();
	}
	public static boolean inAuto(){
		return in_auto;
	}

	public static void  endAuto(){
		in_auto=false;
		simulation.endAuto();
	}
	
}
