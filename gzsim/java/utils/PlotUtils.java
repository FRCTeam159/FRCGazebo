
package utils;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PlotUtils {
    public static enum UnitType {
        FEET, METERS, INCHES, DEGREES, RADIANS
    }

    public static int PLOT_NONE = 0;
    public static int PLOT_DISTANCE = 1;
    public static int PLOT_DYNAMICS = 2;
    public static int PLOT_POSITION = 3;
    public static int PLOT_CALIBRATE  = 4;
    public static int PLOT_GENERIC  = 5;

    public static int auto_plot_option = PLOT_NONE;

    private static NetworkTable table = null;
    private static NetworkTableEntry newPlot;
    private static NetworkTableEntry plotData;
    private static UnitType distance_units_type = UnitType.METERS; // assumes input data is in meters (keep)
    private static UnitType angle_units_type = UnitType.RADIANS; // assumes input data is in degrees (convert)

    private static PathData last_data;
    private static double lastVelocity = 0;
    private static double last_heading = 0;

    private static PathData data_sum;

    private static Averager acc_average = new Averager(20);
    private static Averager vel_average = new Averager(10);

    static ArrayList<PathData> data = new ArrayList<>();

    private static int plotCount = 0;
    private static int data_count = 0;

    
    public static void setDistanceUnits(UnitType t) {
        distance_units_type = t;
    }
    public static void setAngleUnits(UnitType t) {
        angle_units_type = t;
    }
    
    public static void initPlot() {
        acc_average.reset();
        vel_average.reset();
        lastVelocity = 0;
        last_heading = 0;
        data_sum = new PathData();
        last_data = new PathData();
        data_count = 0;
    }
    public static void setInitialPose(Pose2d pose, double trackwidth){
        last_data = getPathPosition(0, pose, trackwidth);
    }

    // =================================================
    // getPathPosition return wheel positions from pose
    // =================================================
    public static PathData getPathPosition(double tm, Pose2d pose, double chassis_width) {
        PathData pd = new PathData();
        double x = pose.getX();
        double y = pose.getY();

        Rotation2d heading = pose.getRotation();
        double cos_angle = heading.getCos();
        double sin_angle = heading.getSin();
        double w = 0.5 * chassis_width;

        double lx = x - (w * sin_angle);
        double ly = y + (w * cos_angle);
        double rx = x + (w * sin_angle);
        double ry = y - (w * cos_angle);

        pd.tm = tm;
        pd.d[0] = distanceUnits(lx); // left
        pd.d[1] = distanceUnits(ly);
        pd.d[2] = distanceUnits(x); // center
        pd.d[3] = distanceUnits(y);
        pd.d[4] = distanceUnits(rx); // right
        pd.d[5] = distanceUnits(ry);
        return pd;
    }

    // =================================================
    // plotPosition: plot the deviation between a target and observed wheel positions
    // - converts position info an xy plot to display wheel positions
    // =================================================
    // inputs:
    //   double tm time of sample
    //   Pose2d target_pose expected pose
    //   Pose2d current_pose observed pose
    //   Pose2d current_pose observed pose
    //   double trackwidth left-right wheel base distance of robot
    // outputs:
    //   pd.tm time of sample
    //   pd.d[0] expected left y vs x
    //   pd.d[1] observed left y vs x
    //   pd.d[2] expected center y vs x
    //   pd.d[3] observed center y vs x
    //   pd.d[4] expected right y vs x
    //   pd.d[5] observed right y vs x
    // =================================================
    public static PathData plotPosition(double tm, Pose2d target_pose, Pose2d current_pose, double trackwidth) {
        PathData pd = new PathData();
        PathData t_data = getPathPosition(tm, target_pose, trackwidth);
        PathData d_data = getPathPosition(tm, current_pose, trackwidth);

        pd.d[0] = Math.abs(d_data.d[0]);
        pd.d[1] = d_data.d[1];
        pd.d[2] = Math.abs(t_data.d[0]);
        pd.d[3] = t_data.d[1];

        //System.out.println(pd.d[0]+" "+pd.d[1]+" "+pd.d[2]+" "+pd.d[3]);

        pd.d[4] = Math.abs(d_data.d[2]);
        pd.d[5] = d_data.d[3];
        pd.d[6] = Math.abs(t_data.d[2]);
        pd.d[7] = t_data.d[3];

        pd.d[8] = Math.abs(d_data.d[4]);
        pd.d[9] = d_data.d[5];
        pd.d[10] = Math.abs(t_data.d[4]);
        pd.d[11] = t_data.d[5];

        return pd;
    }

    // =================================================
    // plotDistance: collect DathData objects for motion error plot
    // - converts position info into distance traveled
    // =================================================
    // inputs:
    //  double tm time of sample
    //  Pose2d pose expected pose
    //  double ld observed left side distance
    //  double rd observed right side distance
    //  double gh observed heading
    // outputs:
    //  pd.tm time of sample
    //  pd.d[0] expected left distance
    //  pd.d[1] observed left distance
    //  pd.d[2] expected right distance
    //  pd.d[3] observed right distance
    //  pd.d[4] expected heading
    //  pd.d[5] observed heading
    // =================================================
  
    public static PathData plotDistance(double tm, Pose2d pose, double curve, double ld, double rd, double gh,double trackwidth) {
        PathData pd = new PathData();

        PathData cd = getPathPosition(tm, pose, trackwidth);
        PathData delta = cd.minus(last_data);
        double x = delta.d[0];
        double y = delta.d[1];
        double d = Math.sqrt(x * x + y * y);
        double cl=d*(1+0.2*Math.abs(curve));
        data_sum.d[0]+=cl;
        x = delta.d[4];
        y = delta.d[5];
        d = Math.sqrt(x * x + y * y);
        double cr=d*(1+0.2*Math.abs(curve));
        data_sum.d[1]+=cr;

        //System.out.println("cl:"+cl+" cr:"+cr+" curve:"+curve);

        double ch = pose.getRotation().getDegrees();
        ch = ch > 180 ? ch - 360 : ch; // convert to signed angle fixes problem:th 0->360 gh:-180->180

        gh = unwrap(last_heading, gh);

        pd.tm = tm;
        pd.d[0] = ld;
        pd.d[1] = data_sum.d[0];
        pd.d[2] = rd;
        pd.d[3] = data_sum.d[1];
        pd.d[4] = angleUnits(gh);
        pd.d[5] = angleUnits(ch);

        last_heading = gh;
        last_data = cd;

        return pd;
    }

    // =================================================
    // plotDynamics: collect DathData objects for dynamics error plot
    // =================================================
    // inputs:
    //  double tm        time of samnple
    //  Pose2d target    target pose 
    //  Pose2d current   observed pose 
    //  double tv        target velocity
    //  double v         observed velocity
    // outputs:
    //  pd.tm time of sample
    //  pd.d[0] current distance from start traveled
    //  pd.d[1] target distance from start traveled
    //  pd.d[2] current velocity
    //  pd.d[3] target velocity
    //  pd.d[4] current acceleration
    //  pd.d[5] target acceleration
    // =================================================
    public static PathData plotDynamics(double tm,
      Pose2d target, Pose2d current, double tv, double v, double ta) {
        PathData pd = new PathData();
        double x=distanceUnits(current.getX());
        double y=distanceUnits(current.getY());
        double obs_distance = Math.sqrt(x * x + y * y);

        x = distanceUnits(target.getX());
        y = distanceUnits(target.getY());
        double exp_distance = Math.sqrt(x * x + y * y);

        double acceleration = 0;
        double a = 0;
        double vel = vel_average.getAve(distanceUnits(v));

        if (data_count > vel_average.numAves())
            acceleration = (vel - lastVelocity) / 0.02;

        a = acc_average.getAve(acceleration);

        pd.tm = tm;
        pd.d[0] = obs_distance;
        pd.d[1] = exp_distance;
        pd.d[2] = v;
        pd.d[3] = tv;
        pd.d[4] = a;
        pd.d[5] = distanceUnits(ta);
        lastVelocity = vel;
        data_count++;
        return pd;
    }

    public static double metersToFeet(double meters) {
        return meters * 100 / (2.54 * 12);
    }
    public static double degreesToRadians(double t) {
        return t * 2 * Math.PI / 360.0;
    }
    
    // convert to requested distance units type
    // assumes input data is in meters
    public static double distanceUnits(double f) {
        if (distance_units_type == UnitType.INCHES)
            return 12.0 * metersToFeet(f);
        else if (distance_units_type == UnitType.FEET)
            return metersToFeet(f);
        else
            return f;
    }
    // convert to requested angle units type
    // assumes input data is in degrees
    public static double angleUnits(double f) {
        if (angle_units_type == UnitType.RADIANS)
            return degreesToRadians(f);
        else
            return f;
    }
    // removes heading discontinuity at 180 degrees
    public static double unwrap(double previous_angle, double new_angle) {
        double d = new_angle - previous_angle;
        d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
        return previous_angle + d;
    }
/*
    // Plot Path motion (values vs time)
    public static void genericPlot(ArrayList<PathData> d, String label_list[], int traces) {
        JFrame frame = new PlotRenderer(d, traces, PlotRenderer.TIME_MODE, label_list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }

    // Plot Path motion (wheel traces y vs x)
    public static void genericXYPlot(ArrayList<PathData> d, String label_list[], int traces) {
        JFrame frame = new PlotRenderer(d, traces, PlotRenderer.XY_MODE, label_list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
  */  
    // publish plot data to NetworkTables
    public static void publish(ArrayList<PathData> dataList, int traces, int type) {
        if (table == null) {
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            table = inst.getTable("plotdata");
            newPlot = table.getEntry("NewPlot");
            plotData = table.getEntry("PlotData");
        }
        double info[] = new double[4];
        int points = dataList.size();
        info[0] = plotCount;
        info[1] = traces;
        info[2] = points;
        info[3] = type;

        System.out.println("Publishing Plot Data "+plotCount+" points:"+points+" traces:"+traces);

        newPlot.setDoubleArray(info);
        if(type==PLOT_POSITION)
            traces*=2;
        for (int i = 0; i < points; i++) {
            PathData pathData = dataList.get(i);
            double data[] = new double[traces + 2];
            data[0] = (double) i;
            data[1] = pathData.tm;
            for (int j = 0; j < traces; j++) {
                data[j + 2] = pathData.d[j];
            }
            plotData = table.getEntry("PlotData"+i);
            plotData.setDoubleArray(data);
        }
        dataList.clear();
        plotCount++;
    }
}
