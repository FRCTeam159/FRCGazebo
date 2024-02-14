
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.CombinedRuntimeLoader;

import java.io.IOException;
import java.util.ArrayList;

import javax.swing.SwingUtilities;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.CameraServerJNI;
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
        CameraServerJNI.Helper.setExtractOnStaticLoad(false);
        CameraServerCvJNI.Helper.setExtractOnStaticLoad(false);

        CombinedRuntimeLoader.loadLibraries(Program.class, "wpiutiljni", "wpimathjni", "ntcorejni", Core.NATIVE_LIBRARY_NAME, "cscorejni");
        PlotServer plotter = new PlotServer();
		plotter.run();
    }
    public void run() {
		System.out.println("starting Plot Server<"+mode+">");
		var inst = NetworkTableInstance.getDefault();
		
		table = inst.getTable("plotdata");

		DoubleArraySubscriber newPlotSub = table.getDoubleArrayTopic("NewPlot").subscribe(null);
		DoubleArraySubscriber dataSub = table.getDoubleArrayTopic("PlotData").subscribe(null);

        inst.startClient4("plot client");
    	inst.setServer("localhost");

		while (true) {
			try {
				//System.out.println("PlotServer waiting for data");
				Thread.sleep(1000);
				
				double[] info=newPlotSub.get();
				if(info!=null){
					setNewPlot(info);
				}
				if(newplot){
					double[] data=dataSub.get();
					if(data !=null){
						setPlotData(data);
						newplot=false;
					}
				}

			} catch (Exception ex) {
				System.out.println("exception " + ex);
			}
		}
	}
    private void setPlotData(double[] data) {
		System.out.println("PlotData:" + id + " " + traces + " " + data.length);
		int ptraces=traces;
		if(type==PlotUtils.PLOT_POSITION)
			ptraces*=2;
		for(int i=0;i<points;i++){
			PathData pd = new PathData();
			int k=i*(ptraces+2);
			index = (int) data[k+0];
			pd.tm = data[k+1];
			for (int j = 0; j < ptraces; j++) {
				pd.d[j] = data[k+j + 2];
			}
			list.add(pd);
		}
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				createAndShowGui(list, traces, type);
			}
		});
	}

    void setNewPlot(double[] info){
		if(info.length==0){
			System.out.println("Error empty plot !");
			return;
		}
		list.clear();
		int plot_id=(int) info[0];
		if(plot_id !=id){
			id = (int) info[0];
			traces = (int) info[1];
			points = (int) info[2];
			type = (int) info[3];
			index = 0;
			count = 0;
			System.out.println("NewPlot id:" + id + " traces:" + traces + " points:" + points+" type:"+type);
			newplot=true;
		}
		else
			newplot=false;		
	}
	private void createAndShowGui(ArrayList<PathData> d, int traces, int type) {
		System.out.println("Showing plot: Size:" + d.size()+" traces:"+traces+" type:"+type);
		PlotRenderer.showPlot(d, traces, type);
		list = new ArrayList<PathData>();
	}
	

}
