
package utils;

import java.util.ArrayList;

import javax.swing.JFrame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PlotUtils {
    public static enum UnitType {
        FEET,METERS,INCHES
    }
    public static int PLOT_NONE = 0;
    public static int PLOT_DISTANCE = 1;
    public static int PLOT_DYNAMICS = 2;
    public static int PLOT_POSITION = 3;

    public static int auto_plot_option=PLOT_NONE;

    private static NetworkTable table=null;
    private static NetworkTableEntry newPlot;
    private static NetworkTableEntry plotParams;
    private static NetworkTableEntry plotData;
    private static UnitType units_type=UnitType.METERS; // assumes Trajectory data is in meters

    private ArrayList<PathData> pathDataList = new ArrayList<>();
    static ArrayList<PathData> data = new ArrayList<>();

    private static int plotCount = 0;

    public static double metersToFeet(double meters) {
        return meters * 100 / (2.54 * 12);
    }
    public static void setUnits(UnitType t){
        units_type=t;
    }
    public static double units(double f){
        if(units_type==UnitType.INCHES)
            return 12.0*metersToFeet(f);
        else if(units_type==UnitType.FEET)
            return metersToFeet(f);
        else 
            return f;
    }
    public void addPlotData(PathData data){
        pathDataList.add(data);

    }
    // publish plot data to NetworkTables
    public static void publish(ArrayList<PathData> dataList, int traces) {
        if(table==null){
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            table = inst.getTable("plotdata");
            newPlot = table.getEntry("NewPlot");
            plotParams = table.getEntry("PlotParams" + plotCount);
            plotData = table.getEntry("PlotData");
        }
        double info[] = new double[4];
        int points = dataList.size();
        info[0] = plotCount;
        info[1] = traces;
        info[2] = points;
        info[3] = 3;

        System.out.println("Publishing Plot Data");
    
        newPlot.setNumber(plotCount); 
        plotParams.setDoubleArray(info);

        for (int i = 0; i < points; i++) {
            PathData pathData = dataList.get(i);
            double data[] = new double[traces + 2];
            data[0] = (double) i;
            data[1] = pathData.tm;
            for (int j = 0; j < traces; j++) {
                data[j + 2] = pathData.d[j];
            }
            plotData.setDoubleArray(data);
        }
        dataList.clear();
        plotCount++;
    }
    
    // convert a Plot structure to display wheel positions
    public static PathData getPathPosition(double tm,Pose2d pose, double chassis_width){
        PathData pd = new PathData();
        double x = pose.getX();
        double y = pose.getY();

        Rotation2d heading = pose.getRotation();
        double cos_angle = heading.getCos();
        double sin_angle = heading.getSin();
        double w =0.5*chassis_width;

        double lx = x - (w * sin_angle);
        double ly = y + (w * cos_angle);
        double rx = x + (w * sin_angle);
        double ry = y - (w * cos_angle);

        pd.tm = tm;
        pd.d[0] = units(lx); // left
        pd.d[1] = units(ly);
        pd.d[2] = units(x);  // center
        pd.d[3] = units(y);
        pd.d[4] = units(rx); // right
        pd.d[5] = units(ry);
        return pd;
    }
    
    // Plot Path motion (values vs time)
    public static void genericPlot(ArrayList<PathData> d, String label_list[], int traces ) {
        JFrame frame = new PlotRenderer(d, traces, PlotRenderer.TIME_MODE, label_list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
    // Plot Path motion (wheel traces y vs x)
    public static void genericXYPlot(ArrayList<PathData> d, String label_list[], int traces ) {
        JFrame frame = new PlotRenderer(d, traces, PlotRenderer.XY_MODE, label_list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }

}
