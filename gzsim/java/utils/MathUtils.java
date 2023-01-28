package utils;

public class MathUtils {
    public static double lerp(double x, double xmin, double xmax, double ymin, double ymax){
        x=x<xmin?xmin:x;
        x=x>xmax?xmax:x;
        double a=(ymax-ymin)/(xmax-xmin);
        double b=ymin-a*xmin;
        double y=a*x+b;
        return y;
    } 
}
