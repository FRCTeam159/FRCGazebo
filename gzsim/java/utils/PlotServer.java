package utils;

import java.util.ArrayList;

import javax.swing.SwingUtilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;

public class PlotServer implements TableEntryListener {
	ArrayList<PathData> list = new ArrayList<PathData>();
	int traces = 0;
	int index = 0;
	int points = 0;
	int count = 0;
	int id = 0;boolean debug=false;

	int type = PlotUtils.PLOT_GENERIC;
	String mode=null;

	NetworkTable table;
	public boolean server_running = false;
	static NetworkTableInstance inst;

	public static void main(String[] args) {
		inst = NetworkTableInstance.getDefault();
		PlotServer plotter;
		if(args.length>0)
			plotter = new PlotServer(args[0]);
		else
			plotter = new PlotServer("client");
		plotter.run();
	}

	public PlotServer(String h) {
		mode=h;
	}

	public void run() {
		System.out.println("starting Plot Server<"+mode+">");
		inst = NetworkTableInstance.getDefault();
		if(mode != null && mode == "server")
			inst.startServer();
		else
			inst.startClient("localhost");
		table = inst.getTable("plotdata");
		table.getEntry("NewPlot");
		table.getEntry("PlotData");
		table.addEntryListener(this, kUpdate | kNew);
		while (true) {
			try {
				//System.out.println("PlotServer waiting for data");
				Thread.sleep(1000);
			} catch (Exception ex) {
				System.out.println("exception " + ex);
			}
		}
	}

	private void createAndShowGui(ArrayList<PathData> d, int traces, int type) {
		System.out.println("Showing plot: Size:" + d.size()+" traces:"+" type:"+type);
		PlotRenderer.showPlot(d, traces, type);
		list = new ArrayList<PathData>();
	}

	@Override
	public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
			int flags) {
		if (key.equals("NewPlot")) {
			list.clear();
			double info[] = entry.getDoubleArray(new double[0]);
			if(info.length==0){
				System.out.println("Error empty plot !");
			}
			else{
				id = (int) info[0];
				traces = (int) info[1];
				points = (int) info[2];
				type = (int) info[3];
				index = 0;
				count = 0;
				if(debug)
					System.out.println("NewPlot id:" + id + " traces:" + traces + " points:" + points+" type:"+type);
			}
		}
		if (key.equals("PlotData" + count)) {
			double data[] = entry.getDoubleArray(new double[0]);
			if (data.length == 0) {
				System.out.println("Error: recieved empty data packet !!");
			} else {
				PathData pd = new PathData();

				index = (int) data[0];
				pd.tm = data[1];
				for (int i = 0; i < data.length - 2; i++) {
					pd.d[i] = data[i + 2];
				}
				list.add(pd);
			}
			count++;
			if(debug)
				System.out.println("PlotData:" + id + " " + index + " " + list.size() + " " + data.length);
		}
		if (count == points && points > 0) {
			count = 0;
			index = 0;
			SwingUtilities.invokeLater(new Runnable() {
				public void run() {
					createAndShowGui(list, traces, type);
				}
			});
		}
	}
}
