package utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class TagTarget {
    Pose3d targetPose;
    public static double targetSize=0.375;
    int targetID=0;

    public TagTarget( int id, Pose3d pose) {
        targetPose = pose;
        targetID = id;
    }

    public TagTarget(int id, double x, double y, double z, double a) {
        targetPose = new Pose3d(x,y,z,new Rotation3d(0,0,a));
        targetID = id;
    }
    
    public int getID(){ 
        return targetID;
    }
    public Pose3d getPose(){
        return targetPose;
    }
    public double getTargetSize(){
        return targetSize;
    }
    public void setTargetSize(double s){
        targetSize=s;
    }
   
}
