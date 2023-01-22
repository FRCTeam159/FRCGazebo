package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.PlotServer;

public class PlotServerTest {

	NetworkTableInstance inst;

	public static void main(String[] args) {
		PlotServer plotter;
		if(args.length>0)
			plotter = new PlotServer(args[0]);
		else
			plotter = new PlotServer("client");
		plotter.run();
	}	
}
