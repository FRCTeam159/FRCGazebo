/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

import org.opencv.core.RotatedRect;

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.util.ArrayList;
import frc.robot.objects.Camera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

import gazebo.SimRangefinder;

/**
 * Add your docs here.
 */
public class VisionProcess extends Thread {
  static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
  
  //public static double cameraFovW =  41.8;//
  public static double cameraFovW =  Math.toDegrees(1.0471975511966);//

  public static double imageWidth = 320;
  public static double imageHeight = 240;
  //double angleFactorWidth = Math.toDegrees(cameraFovW) / imageWidth;
  double angleFactorWidth = cameraFovW / imageWidth;

  SimRangefinder rangefinder = new SimRangefinder(1);
  double range_offset=10.0; // distance from rangefinder to bumpers
  
  double min_range = 35;
  double max_range = 70;
  double range = 0;
  double targetAngle = 0;
  double targetOffset = 0;

  Camera camera;
  //TargetInfo target;

  public double getRange() {
    double meters = rangefinder.getDistance();
    // if(meters>=1)
    //   return 1234;
    // double inches=39.3701 * meters;
    // return inches-range_offset;
    return meters;
  }

  public void init() {
    SmartDashboard.putNumber("Targets", 0);
    SmartDashboard.putNumber("H offset", 0);
    SmartDashboard.putNumber("Target Tilt", 0);
    SmartDashboard.putNumber("Range", 0);
    SmartDashboard.putBoolean("Show HSV", false);
    
    System.out.println("fov " + Math.toDegrees(cameraFovW));
  }

  double round10(double x) {
    return Math.round(x * 10 + 0.5) / 10.0;
  }

  public void run() {
    camera=new Camera(0); // 9000
    
    GripPipeline grip = new GripPipeline();
    CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);
    Mat mat = new Mat();
    ArrayList<RotatedRect> rects = new ArrayList<RotatedRect>();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("TargetData");

    edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
    timer.start();

    while (true) {
      try {
        Thread.sleep(20);
      } catch (InterruptedException ex) {
        System.out.println("exception)");
      }
      timer.reset();
      //if (!vcap.read(mat)) 
      mat=camera.getFrame();
      if(mat==null)
        continue;
      
      double dt = timer.get() * 1000;
      grip.process(mat);

      Boolean show_hsv = SmartDashboard.getBoolean("Show HSV", false);
      if (show_hsv) {
        //Mat hsv = grip.hsvThresholdOutput(); // display HSV image
        //hsv.copyTo(mat);
        mat = grip.hsvThresholdOutput();
      }
      ArrayList<MatOfPoint> contours = grip.filterContoursOutput();
      rects.clear();
      double max_area = 0;
      RotatedRect biggest = null;
      // find the bounding boxes of all targets
      // public static RotatedRect minAreaRect(MatOfPoint2f points)
      // Imgproc.boxPoints(RotatedRect box, Mat points)
      for (int i = 0; i < contours.size(); i++) {
        MatOfPoint contour = contours.get(i);
        double area = Imgproc.contourArea(contour);
        MatOfPoint2f NewMtx = new MatOfPoint2f(contour.toArray());
        RotatedRect r = Imgproc.minAreaRect(NewMtx);
        if (area > max_area) {
          biggest = r;
          max_area = area;
        }
        rects.add(r);
      }
      // calculate distance to target
      // - using ht
      SmartDashboard.putNumber("Targets", rects.size());
      if (biggest != null) {
        double h = biggest.size.height;
        double w = biggest.size.width;
        targetAngle = biggest.angle;

        if (biggest.size.width > biggest.size.height) {
          h = biggest.size.width;
          w = biggest.size.height;
          targetAngle += 90.0;
        }
        Point ctr = biggest.center;
        targetOffset = angleFactorWidth * (ctr.x - 0.5 * imageWidth);
        SmartDashboard.putNumber("H offset", round10(targetOffset));
        SmartDashboard.putNumber("Target Tilt", round10(targetAngle));
      }
      double range = getRange();
      SmartDashboard.putNumber("Range", round10(range));

      table.getEntry("Range").setDouble(range);
      table.getEntry("Target Tilt").setDouble(targetAngle);
      table.getEntry("H offset").setDouble(targetOffset);
      table.getEntry("Targets").setDouble(rects.size());

      for (int i = 0; i < rects.size(); i++) {
        RotatedRect r = rects.get(i);
        Point[] vertices = new Point[4];
        r.points(vertices);
        for (int j = 0; j < 4; j++) {
          if (r == biggest)
            Imgproc.line(mat, vertices[j], vertices[(j + 1) % 4], new Scalar(255, 0, 0), 2);
          else
            Imgproc.line(mat, vertices[j], vertices[(j + 1) % 4], new Scalar(255, 255, 0), 1);
        }
      }
      outputStream.putFrame(mat);
    }
   // Process();
  }
  // void Process() {
  //   GetTargetInfo();
  //   PublishTargetInfo();
  // }
  // class TargetInfo {
	// 	Point Center;
	// 	double Distance;
	// 	double HorizontalOffset;
	// 	double HorizontalAngle;
	// 	double Height;
	// 	double Width;
	// 	double ActualWidth;
	// 	double ActualHeight;
	// 	int numrects;

  //   void CalcTargetInfo(int n,Point top, Point bottom) {
  //     Height=bottom.y-top.y; // screen y is inverted
  //     Width=bottom.x-top.x;
  //     Center.x=0.5*(bottom.x+top.x);
  //     Center.y=0.5*(bottom.y+top.y);
  //     HorizontalOffset=Center.x-imageWidth/2;
  //     ActualHeight=5.0;
  //     ActualWidth=(n==1?2.0:10.25);	//inches
  //     Distance=cameraInfo.fovFactor*ActualHeight/Height;
  //   }
	// };
}
