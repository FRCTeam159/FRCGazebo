
package objects;

import org.opencv.core.Point;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.apriltag.AprilTagDetection;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;

public class AprilTag {
    int id;
    int tag_id;
    double margin;
    double centerX, centerY;
    double[][] corners;
    double[][] homog;

    Transform3d poseResult;

    public AprilTag(AprilTagDetection det,Transform3d pose){
        double[] c=det.getCorners();
        int k=0;
        corners=new double[4][2];
        for(int i=0;i<4;i++)
        for(int j=0;j<2;j++)
            corners[i][j]=c[k++];
        tag_id=det.getId();
        margin=det.getDecisionMargin();
        centerX=det.getCenterX();
        centerY=det.getCenterY();
        c=det.getHomography();
        k=0;
        homog=new double[3][3];
        for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
          homog[i][j]=c[k++];

        poseResult=pose;
    }
    
    private static final Rotation3d WPILIB_BASE_ROTATION = new Rotation3d(
        new MatBuilder<>(Nat.N3(), Nat.N3()).fill(0, 1, 0, 0, 0, 1, 1, 0, 0));

    public static Pose3d convertOpenCVtoWPIlib(Transform3d cameraToTarget3d) {
        Pose3d p=new Pose3d(cameraToTarget3d.getTranslation(),cameraToTarget3d.getRotation());
        var nwu = CoordinateSystem.convert(p, CoordinateSystem.EDN(), CoordinateSystem.NWU());
        return new Pose3d(nwu.getTranslation(), WPILIB_BASE_ROTATION.rotateBy(nwu.getRotation()));
    }

    public int getId() {
        return id;
    }

    public int getTagId() {
        return tag_id;
    }

    public void setCenterPoint(double x, double y) {
        centerX = x;
        centerY = y;
    }

    public double getDecisionMargin() {
        return margin;
    }

    public double width() {
        return corners[1][0] - corners[0][0];
    }

    public double height() {
        return corners[0][1] - corners[3][1];
    }

    public double getCenterX() {
        return centerX;
    }

    public Point center() {
        return new Point(centerX, centerY);
    }

    public Point tl() {
        return new Point(corners[0][0], corners[0][1]);
    }

    public Point br() {
        return new Point(corners[2][0], corners[2][1]);
    }

    public Point tr() {
        return new Point(corners[1][0], corners[1][1]);
    }

    public Point bl() {
        return new Point(corners[3][0], corners[3][1]);
    }

    public double getCenterY() {
        return centerY;
    }

    public double[][] getCorners() {
        return corners;
    }

    public double[][] getHomog() {
        return homog;
    }

    public Transform3d getPoseTransform() {
        return poseResult;
    }
    public Pose3d getPose() {
        return convertOpenCVtoWPIlib(poseResult);
    }

    public double getDistance() {
		// camera to target distance along the ground
		Translation2d trans=getPose().getTranslation().toTranslation2d();
		double distance = trans.getNorm(); 
        return distance;
    }
    public double getYaw() {
        Pose3d pose=getPose();
        double angle=Math.atan2(pose.getTranslation().getY(), pose.getTranslation().getX());
		return Math.toDegrees(angle);	
    }
    public double getPitch() {
        Pose3d pose=getPose();
        double angle=Math.atan2(pose.getTranslation().getZ(), pose.getTranslation().getX());
		return Math.toDegrees(angle);	
    }
    
    public double getX(){
        return getPose().getTranslation().getX();
    }
    public double getY(){
        return getPose().getTranslation().getY();
    }
    public double getZ(){
        return getPose().getTranslation().getZ();
    }
    public String toString() {
        String str=null;
        Pose3d pose=getPose();
        
        if(pose !=null)
          str = String.format("id:%d X:%-2.1f Y:%-2.1f Z:%-2.1f H:%-2.1f P:%-2.1f",
                tag_id, getX(), getY(), getZ(), getYaw(), getPitch());
        return str;
    }

    public void print() {
        String str = toString();
        System.out.println(str);
    }
}