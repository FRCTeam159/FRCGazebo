// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.Core;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import objects.Camera;

import objects.AprilTag;

public class TagDetector extends SubsystemBase {
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }
  static Camera cam;

  static AprilTagDetector wpi_detector;

  static AprilTagPoseEstimator.Config wpi_poseEstConfig;
  static AprilTagPoseEstimator wpi_pose_estimator;
  static boolean m_targeting = false;
  
  static AprilTag[] tags = null;

  public static double min_decision_margin = 30; // reject tags less than this

  public TagDetector() {
    cam = new Camera(0, DualCameras.kCamWidth, DualCameras.kCamHeight, 40); // specs for Gazebo camera
    wpi_detector = new AprilTagDetector();
    try{
      wpi_detector.addFamily("tag16h5", 0);
      wpi_detector.addFamily("tag36h11", 0);
      System.out.println("2 tag Families loaded");
    } catch (Exception ex){
      System.out.println("TagDetector exception:" + ex);
    }
    wpi_poseEstConfig = new AprilTagPoseEstimator.Config(TargetMgr.targetSize, cam.fx, cam.fy, cam.cx, cam.cy);
    wpi_pose_estimator = new AprilTagPoseEstimator(wpi_poseEstConfig);

    cam.start();
  }

  public static Mat getFrame() {
    Mat mat = cam.getFrame();
    if (mat == null)
      return null;

    if(!DualCameras.kTagCamera)
      return mat;

    boolean autoselect = Autonomous.getAutoset();
    boolean usetags = Autonomous.getUsetags();
    boolean showtags = Autonomous.getShowtags();

    tags = null;

    if (m_targeting || showtags || (autoselect && !TargetMgr.startPoseSet() && usetags)) {
      tags = getTags(mat);
      if (tags != null && tags.length > 1) {
        Arrays.sort(tags, new SortbyDistance());
      }
    }

    // set initial starting position and alliance

    if (autoselect && !TargetMgr.startPoseSet()) {
      if (usetags) {
        if (tags != null && tags.length == 2)
          TargetMgr.setStartPose(tags);
      } else {
        int alliance = Autonomous.getAlliance();
        int position = Autonomous.getPosition();
        TargetMgr.setTarget(alliance, position);
      }
    }
    if (tags != null)
      showTags(tags, mat);
    return mat;
  }
  // targeting methods
  public static void setTargeting(boolean state) {
   // System.out.println("SET TARGETING " + state);
    m_targeting = state;
  }

  public static AprilTag[] getTags() {
    return tags;
  }

  public static boolean isTargeting() {
    return m_targeting;
  }

  static void showTags(AprilTag[] tags, Mat mat) {
    for (int i = 0; i < tags.length; i++) {
      AprilTag tag = tags[i];

      Point c = tag.center();
     
      Scalar lns = new Scalar(255.0, 255.0, 0.0);
      Imgproc.line(mat, tag.tl(), tag.tr(), lns, 2);
      Imgproc.line(mat, tag.tr(), tag.br(), lns, 2);
      Imgproc.line(mat, tag.br(), tag.bl(), lns, 2);
      Imgproc.line(mat, tag.bl(), tag.tl(), lns, 2);

      // Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 0.0), 2);
      //Imgproc.drawMarker(mat, c, new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 35, 2, 8);
      Point p = new Point(tag.bl().x - 10, tag.bl().y - 10);
      Imgproc.putText(
          mat, // Matrix obj of the image
          "[" + tag.getTagId() + "]", // Text to be added
          p, // point
          Imgproc.FONT_HERSHEY_SIMPLEX, // front face
          1, // front scale
          new Scalar(255, 0, 0), // Scalar object for color
          2 // Thickness
      );
      if(i==TargetMgr.kBestTarget && m_targeting){
        c.y+=TargetMgr.kVertOffset*DualCameras.kCamHeight;
        c.x+=TargetMgr.kHorizOffset*DualCameras.kCamWidth;
        Imgproc.drawMarker(mat, c, new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 35, 2, 8);
      }
    }
  }

  // return an array of tag info structures from an image
  private static AprilTag[] getTags(Mat mat) {
    AprilTag[] atags = null;
    Mat graymat = new Mat();
    Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
    AprilTagDetection[] detections = wpi_detector.detect(graymat);

    // reject tags with a poor decision margin or out of expected index range
    List<AprilTagDetection> list = new ArrayList<AprilTagDetection>();
    for (int i = 0; i < detections.length; i++) {
      AprilTagDetection dect = detections[i];
      int id = dect.getId();
      if (id < TargetMgr.minTargetId() || id > TargetMgr.maxTargetId())
        continue;
      if (dect.getDecisionMargin() > min_decision_margin)
        list.add(dect);
    }

    int num_tags = list.size();
    if (num_tags == 0)
      return null;

    atags = new AprilTag[num_tags];
    for (int i = 0; i < num_tags; i++) {
      AprilTagDetection detection = list.get(i);
      Transform3d pose = wpi_pose_estimator.estimate(detection);
      atags[i] = new AprilTag(detections[i], pose);
    }
    return atags;
  }

  // Helper class extending Comparator interface
  // sort by distance (closest on top)
  static class SortbyDistance implements Comparator<AprilTag> {
    public int compare(AprilTag p1, AprilTag p2) {
      double d1 = p1.getDistance();
      double d2 = p2.getDistance();
      if (d1 < d2)
        return -1;
      if (d1 > d2)
        return 1;
      return 0;
    }
  }
}
