
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.CombinedRuntimeLoader;

import java.io.IOException;
import java.util.ArrayList;

import javax.swing.SwingUtilities;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.util.WPIUtilJNI;
import utils.PathData;
import utils.PlotRenderer;
import utils.PlotUtils;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;

/**
 * Program
 */
public class Program {
    ArrayList<PathData> list = new ArrayList<PathData>();
	int traces = 0;
	int index = 0;
	int points = 0;
	int count = 0;
	int id = -1;
	boolean debug=false;

	int type = PlotUtils.PLOT_GENERIC;
	String mode=null;

	static NetworkTable table;
	public boolean server_running = false;

	static boolean newplot=false;

	private static int plotCount = 0;
    public static void main(String[] args) throws IOException {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);

		System.out.println("PlotServer.Program main margs="+args.length);
    
        CombinedRuntimeLoader.loadLibraries(Program.class, "wpiutiljni", "wpimathjni", "ntcorejni");
        PlotServer plotter;
		if(args.length>0)
		 	plotter= new PlotServer(args[0]);
		else
			plotter= new PlotServer("server");
		plotter.run();
    }
    

}
