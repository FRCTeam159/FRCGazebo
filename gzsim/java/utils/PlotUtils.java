
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
        FEET, METERS, INCHES
    }

    public static int PLOT_NONE = 0;
    public static int PLOT_DISTANCE = 1;
    public static int PLOT_DYNAMICS = 2;
    public static int PLOT_POSITION = 3;

    public static int auto_plot_option = PLOT_NONE;

    private static NetworkTable table = null;
    private static NetworkTableEntry newPlot;
    private static NetworkTableEntry plotParams;
    private static NetworkTableEntry plotData;
    private static UnitType units_type = UnitType.METERS; // assumes Trajectory data is in meters

    private static PathData last_data;
    private static double lastVelocity = 0;
    private static double last_heading = 0;

    private static PathData data_sum;

    private static Averager acc_average = new utils.Averager(20);
    private static Averager vel_average = new utils.Averager(10);

    static ArrayList<PathData> data = new ArrayList<>();

    private static int plotCount = 0;
    private static int data_count = 0;

    public static double metersToFeet(double meters) {
        return meters * 100 / (2.54 * 12);
    }

    public static void setUnits(UnitType t) {
        units_type = t;
    }

    public static double units(double f) {
        if (units_type == UnitType.INCHES)
            return 12.0 * metersToFeet(f);
        else if (units_type == UnitType.FEET)
            return metersToFeet(f);
        else
            return f;
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

    // publish plot data to NetworkTables
    public static void publish(ArrayList<PathData> dataList, int traces) {
        if (table == null) {
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
        pd.d[0] = units(lx); // left
        pd.d[1] = units(ly);
        pd.d[2] = units(x); // center
        pd.d[3] = units(y);
        pd.d[4] = units(rx); // right
        pd.d[5] = units(ry);
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

        pd.d[0] = d_data.d[0];
        pd.d[1] = d_data.d[1];
        pd.d[2] = t_data.d[0];
        pd.d[3] = t_data.d[1];

        pd.d[4] = d_data.d[2];
        pd.d[5] = d_data.d[3];
        pd.d[6] = t_data.d[2];
        pd.d[7] = t_data.d[3];

        pd.d[8] = d_data.d[4];
        pd.d[9] = d_data.d[5];
        pd.d[10] = t_data.d[4];
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
    public static PathData plotDistance(double tm, Pose2d pose, double ld, double rd, double gh,double trackwidth) {
        PathData pd = new PathData();

        PathData cd = getPathPosition(tm, pose, trackwidth);
        PathData delta = cd.minus(last_data);
        data_sum = data_sum.plus(delta); // accumulated travel
        // left wheels
        double lx = data_sum.d[0];
        double ly = data_sum.d[1];
        double cl = Math.sqrt(lx * lx + ly * ly);
        // right wheels
        double rx = data_sum.d[4];
        double ry = data_sum.d[5];
        double cr = Math.sqrt(rx * rx + ry * ry);

        double ch = pose.getRotation().getDegrees();
        ch = ch > 180 ? ch - 360 : ch; // convert to signed angle fixes problem:th 0->360 gh:-180->180

        gh = unwrap(last_heading, gh);

        pd.tm = tm;
        pd.d[0] = ld;
        pd.d[1] = cl;
        pd.d[2] = rd;
        pd.d[3] = cr;
        pd.d[4] = d2R(gh);
        pd.d[5] = d2R(ch);

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
        double x=current.getX();
        double y=current.getY();
        double obs_distance = Math.sqrt(x * x + y * y);

        x = target.getX();
        y = target.getY();
        double exp_distance = Math.sqrt(x * x + y * y);

        double acceleration = 0;
        double a = 0;
        double vel = vel_average.getAve(v);

        if (data_count > vel_average.numAves())
            acceleration = (vel - lastVelocity) / 0.02;

        a = acc_average.getAve(acceleration);

        pd.tm = tm;
        pd.d[0] = obs_distance;
        pd.d[1] = exp_distance;
        pd.d[2] = v;
        pd.d[3] = tv;
        pd.d[4] = a;
        pd.d[5] = ta;
        lastVelocity = vel;
        data_count++;
        return pd;
    }

    public static double d2R(double t) {
        return t * 2 * Math.PI / 360.0;
    }

    public static double unwrap(double previous_angle, double new_angle) {
        double d = new_angle - previous_angle;
        d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
        return previous_angle + d;
    }

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

}
