package utils;

import java.util.ArrayList;

import javax.swing.JFrame;
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
	int id = 0;
	NetworkTable table;
	private NetworkTableEntry newPlot;
	private NetworkTableEntry plotData;
	public boolean server_running = false;
	static NetworkTableInstance inst;

	public static void main(String[] args) {
		inst = NetworkTableInstance.getDefault();
		inst.startServer();
		PlotServer plotter = new PlotServer();
		plotter.run();
	}

	public PlotServer() {
	}

	public void run() {
		inst = NetworkTableInstance.getDefault();
		inst.startClient("localhost");
		table = inst.getTable("plotdata");
		newPlot = table.getEntry("NewPlot");
		plotData = table.getEntry("PlotData");
		table.addEntryListener(this, kUpdate | kNew);
		while (true) {
			try {
				System.out.println("PlotSever waiting for data");
				Thread.sleep(1000);
			} catch (Exception ex) {
				System.out.println("exception " + ex);
			}
		}
	}

	private static void createAndShowGui(ArrayList<PathData> list, int traces) {
		JFrame frame = new PlotRenderer(list, traces);
		// frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		System.out.println("Showing plot: Size = " + list.size());
		frame.pack();
		frame.setLocationRelativeTo(null);
		frame.setVisible(true);
		list = new ArrayList<PathData>();
	}

	@Override
	public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
			int flags) {
		if (key.equals("NewPlot")) {
			list.clear();
			double info[] = entry.getDoubleArray(new double[0]);
			id = (int) info[0];
			traces = (int) info[1];
			points = (int) info[2];
			index = 0;
			count = 0;
			System.out.println("NewPlot:" + id + " " + traces + " " + points);
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
			System.out.println("PlotData:" + id + " " + index + " " + list.size() + " " + data.length);
		}
		if (count == points && points > 0) {
			count = 0;
			index = 0;
			SwingUtilities.invokeLater(new Runnable() {
				public void run() {
					createAndShowGui(list, traces);
				}
			});
		}
	}
}
