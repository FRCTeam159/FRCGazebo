/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

import org.opencv.core.RotatedRect;

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.util.ArrayList;
import objects.Camera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.util.Units;
import gazebo.SimRangefinder;

/**
 * Add your docs here.
 */
public class VisionProcess extends Thread implements RobotMap {
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }

  // public static double cameraFovW = 41.8;//
  public static double cameraFovW = Math.toDegrees(1.0471975511966);//
  public static double MIN_RANGE = 2.0;
  public static double imageWidth = 320;
  public static double imageHeight = 240;
  // double angleFactorWidth = Math.toDegrees(cameraFovW) / imageWidth;
  double angleFactorWidth = cameraFovW / imageWidth;

  SimRangefinder rangefinder = new SimRangefinder(FRAME_RANGEFINDER);
  double range_offset = 10.0; // distance from rangefinder to bumpers

  double min_range = 35;
  double max_range = 70;
  static double range = 0;
  static double distance = 0;
  static double VOffset = 0;
  static double HOffset = 0;
  static int targets = 0;

  public static double ActualHeight = Units.inchesToMeters(5.0); // inches
  public static double ActualWidth = Units.inchesToMeters(12.0); // inches


  Camera camera;

  public void init() {
    log();
    System.out.println("fov " + cameraFovW);
    System.out.println("angleFactorWidth " + angleFactorWidth);

  }

  double round10(double x) {
    return Math.round(x * 10 + 0.5) / 10.0;
  }

  public void run() {
    camera = new Camera(1); // 9000

    GripPipeline grip = new GripPipeline();
    CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);
    Mat mat = new Mat();
    ArrayList<RotatedRect> rects = new ArrayList<RotatedRect>();

    edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
    timer.start();

    while (true) {
      try {
        Thread.sleep(10);
      } catch (InterruptedException ex) {
        System.out.println("exception)");
      }
      timer.reset();
      mat = camera.getFrame();
      if (mat == null)
        continue;

      double dt = timer.get() * 1000;
      grip.process(mat);

      Boolean show_hsv = SmartDashboard.getBoolean("Show HSV", false);
      if (show_hsv) {
        mat = grip.hsvThresholdOutput();
        outputStream.putFrame(mat);
        continue;
      }
      ArrayList<MatOfPoint> contours = grip.filterContoursOutput();
      rects.clear();
      double max_area = 0;
      RotatedRect biggest = null;
      // find the bounding boxes of all targets
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
      range = rangefinder.getDistance();
      targets = 0;
      HOffset=VOffset=distance=0;
      if (range > MIN_RANGE && biggest != null) {
        targets = rects.size();
        Point ctr = biggest.center;
        HOffset = angleFactorWidth * (ctr.x - 0.5 * imageWidth);
        VOffset = angleFactorWidth * (ctr.y - 0.5 * imageHeight);
        distance = imageWidth * ActualWidth / biggest.size.width;
      }

      if (targets > 0) {
        for (int i = 0; i < rects.size(); i++) {
          RotatedRect r = rects.get(i);
          Point[] vertices = new Point[4];
          r.points(vertices);
          for (int j = 0; j < 4; j++) {
            if (r == biggest)
              Imgproc.line(mat, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 0, 255), 2);
            else
              Imgproc.line(mat, vertices[j], vertices[(j + 1) % 4], new Scalar(255, 255, 255), 1);
          }
        }
      }
      outputStream.putFrame(mat);
      log();
    }
  }

  public static double getHOffset() {
    return HOffset;
  }

  public static double getVOffset() {
    return VOffset;
  }

  public static double getRange() {
    return range;
  }

  public static double getDistance() {
    return distance;
  }
  public static int getNumTargets() {
    return targets;
  }

  void log() {
    SmartDashboard.putNumber("Targets", targets);
    SmartDashboard.putNumber("H offset", round10(HOffset));
    SmartDashboard.putNumber("V Offset", round10(VOffset));
    SmartDashboard.putNumber("Range", round10(range));
    SmartDashboard.putNumber("Distance", round10(distance));

    SmartDashboard.putBoolean("Show HSV", false);

  }

  public void reset() {
  }

  // void CalcTargetInfo(int n,Point top, Point bottom) {
  // Height=bottom.y-top.y; // screen y is inverted
  // Width=bottom.x-top.x;
  // Center.x=0.5*(bottom.x+top.x);
  // Center.y=0.5*(bottom.y+top.y);
  // HorizontalOffset=Center.x-imageWidth/2;
  // ActualHeight=5.0;
  // ActualWidth=(n==1?2.0:10.25); //inches
  // Distance=cameraInfo.fovFactor*ActualHeight/Height;
  // }
  // };
}
