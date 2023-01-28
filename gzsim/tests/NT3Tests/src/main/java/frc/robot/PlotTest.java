package frc.robot;

import java.util.Random;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PlotTest {
	static int maxDataPoints = 100;
	NetworkTableEntry newPlot;
	NetworkTableEntry plotData;
	NetworkTable table;
	NetworkTableInstance inst;

	public static void main(String [] args) {
		new PlotTest().run();
	}
	public PlotTest(){}
	public void run(){	
		inst = NetworkTableInstance.getDefault();
		//inst.startClient("localhost");
		table = inst.getTable("plotdata");
		newPlot = table.getEntry("NewPlot");
		plotData = table.getEntry("PlotData");	
		int traces=6;
		Random random = new Random();
		int maxScore = 10;
		double info[] = new double[3];
		int cnt=0;
		while (true) {
			double tm=0;
			info[0]=cnt;//plot id
			info[1]=traces; // 1 trace
			info[2]=maxDataPoints;
			try {
				Thread.sleep(10000);
			} catch (InterruptedException ex) {
				System.out.println("exception)");
			}
			newPlot.setDoubleArray(info);

			for (int i = 0; i < maxDataPoints; i++) {
				double data[] = new double[traces+2];

				data[0]=(double)i;
				data[1]=tm;
				for (int j = 0; j < traces; j++) {
					data[j+2]=(double) random.nextDouble() * maxScore;
				}
				plotData = table.getEntry("PlotData"+i);
				plotData.setDoubleArray(data);
				tm+=0.02;
			}			
			cnt++;
		}
	}
}
