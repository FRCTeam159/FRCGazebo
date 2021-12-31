
package utils;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PlotUtils {
    public static enum UnitType {
        FEET,METERS,INCHES
    }
    private static NetworkTable table=null;
    private static NetworkTableEntry newPlot;
    private static NetworkTableEntry plotParams;
    private static NetworkTableEntry plotData;
    private static UnitType units_type=UnitType.METERS; // assumes Trajectory data is in meters

    private static boolean print_trajectory = false;
    private static boolean print_path = false;

    private ArrayList<PathData> pathDataList = new ArrayList<>();
    static ArrayList<PathData> data = new ArrayList<>();

    private static int plotCount = 0;

    public static void setPrintTrajectory(boolean b){
        print_trajectory=b;
    }
    public static void setPrintPath(boolean b){
        print_path=b;
    }
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
    
    // convert a Trajectory state to a Plot structure to display motion
    public static PathData getPathMotion(int i,Trajectory.State state, double chassis_width){
        PathData pd = new PathData();
        double tm = state.timeSeconds;
        double x = state.poseMeters.getX();
        double y = state.poseMeters.getY();

        Rotation2d heading = state.poseMeters.getRotation();
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
        if (print_path)
            System.out.format("%d %f %f %f %f %f %f %f\n", i, tm, lx, ly, x, y, rx, ry);
        return pd;
    }
    
    // convert a Trajectory state to a Plot structure to display dynamics
    public static PathData getPathDynamics(int i,Trajectory.State state){
        PathData pd = new PathData();
        double tm = state.timeSeconds;
        double x = state.poseMeters.getX();
        double y = state.poseMeters.getY();
        double v = state.velocityMetersPerSecond;
        double a = state.accelerationMetersPerSecondSq;
        double h = state.poseMeters.getRotation().getRadians();
        if (print_trajectory)
            System.out.format("%d %f %f %f %f %f %f\n", i, tm, x, y, v, a, h);
        pd.tm = tm;
        pd.d[0] = units(x);
        pd.d[1] = units(y);
        pd.d[2] = units(v);
        pd.d[3] = units(a);
        pd.d[4] = h;
        return pd;
    }

    // Plot Path motion (wheel traces)
    public static void plotCalibration(ArrayList<PathData> d,double p,double v, double a) {
        String label_list[]={"","",""};
    
        label_list[0]=String.format("Position  %1.2f",p);
        label_list[1]=String.format("Velocity  %1.2f",v);
        label_list[2]=String.format("Accel     %1.2f",a);
        JFrame frame = new PlotRenderer(d, 3, PlotRenderer.TIME_MODE, label_list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
    // Plot Path motion (wheel traces)
    public static void plotPathMotion(ArrayList<PathData> d) {
        String label_list[] = 
        { "Left obs", "Left calc", "Right obs", "Right calc","angle obs","angle calc"};
        JFrame frame = new PlotRenderer(d, 6, PlotRenderer.TIME_MODE, label_list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
    // Plot Path motion (wheel traces)
    public static void plotPathMotion(List<Trajectory.State> list, double chassis_width) {
        data.clear();
        for (int i = 0; i < list.size(); i++) {
            Trajectory.State state = list.get(i);
            PathData pd = getPathMotion(i, state,chassis_width);
            data.add(pd);
        }
        String label_list[] = { "Left", "Center", "Right" };
        JFrame frame = new PlotRenderer(data, 3, PlotRenderer.XY_MODE, label_list);
        // frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }

    // Plot Path dynamics
    public static void plotPathDynamics(List<Trajectory.State> list) {
        data.clear();
        for (int i = 0; i < list.size(); i++) {
            Trajectory.State state = list.get(i);
            PathData pd=getPathDynamics(i,state);
            data.add(pd);
        }
        String label_list[] = { "X", "Y", "Velocity", "Acceleration", "Heading" };
        JFrame frame = new PlotRenderer(data, 5, PlotRenderer.TIME_MODE, label_list);
        // frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
