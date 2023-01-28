// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import org.opencv.core.Core;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.objects.Camera;

import frc.robot.objects.AprilTag;

public class TagDetector extends Thread {
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }
  Camera cam;

  public static double maxPoseError = 2;

  protected static CvSource ouputStream;
  protected AprilTagDetector wpi_detector;

  AprilTagPoseEstimator.Config wpi_poseEstConfig;
  AprilTagPoseEstimator wpi_pose_estimator;

  Drivetrain m_drivetrain;

  static double pose_err = 2;
  static AprilTag target_tag = null;
  static public int BEST = -1;

  boolean first = true;

  static Pose2d last_pose = null;
  static int target_id = BEST;

  static double maxtime = 0;
  static double grab_time = 0;
  static double detect_time=0;
  static int count = 0;
  static String test_image = System.getenv("GZ_SIM") + "/docs/apriltag_0_test.jpg";

  public TagDetector(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;

    cam = new Camera(0, 640, 480, 40); // specs for Gazebo camera

    wpi_detector = new AprilTagDetector();
    wpi_detector.addFamily("tag16h5", 0);
    wpi_poseEstConfig = new AprilTagPoseEstimator.Config(TargetMgr.targetSize, cam.fx, cam.fy, cam.cx, cam.cy);
    wpi_pose_estimator = new AprilTagPoseEstimator(wpi_poseEstConfig);

    ouputStream = CameraServer.putVideo("testCamera", cam.image_width, cam.image_height);
    //test();
  }

  // test tag detection jni using an image file
  public void test() {
    Mat mat = Imgcodecs.imread(test_image);
    AprilTag[] tags = getTags(mat);
    for (int i = 0; i < tags.length; i++) {
      tags[i].print();
    }
  }

  // return an array of tag info structures from an image
  private AprilTag[] getTags(Mat mat) {
    AprilTag[] atags;
    Mat graymat = new Mat();
    Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
    AprilTagDetection[] detections = wpi_detector.detect(graymat);

    int num_tags = detections.length;
    atags = new AprilTag[num_tags];
    for (int i = 0; i < num_tags; i++) {
      AprilTagDetection detection = detections[i];
      Transform3d pose = wpi_pose_estimator.estimate(detection);
      atags[i] = new AprilTag(detections[i], pose);
    }
    return atags;
  }

  static public void reset() {
    maxtime = 0;
    grab_time = 0;
    count = 0;
    detect_time=0;
  }

  static public void setTargetId(int i) {
    target_id = i;
  }

  static public void setBestTarget() {
    target_id = BEST;
  }

  static public Pose2d getLastPose() {
    return last_pose;
  }

  // return pose ambiquity
  public static double getPoseError() {
    return pose_err;
  }

  // project a scaler distance and angle to x and y coordinates
  public static Translation2d project(
      double radius, Rotation2d angle) {
    return new Translation2d(angle.getCos() * radius, -angle.getSin() * radius);
  }

  // return position of tag relative to the camera
  public static Pose3d getTargetPose() {
    if (target_tag == null)
      return null;
    return target_tag.getPose();
  }

  //
  // Calculate the position of a tag with respect to the robot
  //
  public static Translation2d camToTarget(AprilTag tag, Rotation2d gyroAngle) {
    Pose3d pose = tag.getPose();
    double distance = tag.getDistance(); // distance to camera
    // convert x,y, coords in camera space to camera angle
    double cam_angle = Math.atan2(pose.getTranslation().getY(), pose.getTranslation().getX());
    // robot pose angle on field = gyro angle - camera angle
    Rotation2d camrot = new Rotation2d(cam_angle);
    Rotation2d rot = gyroAngle.minus(camrot);
    // project camera posion to target onto the field
    return project(distance, rot);
  }

  //
  // Calculate the position of a robot on the field given a visible tag
  // - robot coordinates set to 0,0 at program start
  //
  public static Pose2d getRobotPoseFromTag(AprilTag tag, Rotation2d gyroAngle) {
    TargetMgr.TagTarget target = TargetMgr.getTarget(tag.getTagId());
    if (target == null) { // sometime bad tag id is returned (e.g. 13 vs 0)
      System.out.println("null target:" + tag.getTagId());
      return null;
    }
    Translation2d target_trans = target.getPose().getTranslation().toTranslation2d();
    // get tag location from robot POV
    Translation2d ctot = camToTarget(tag, gyroAngle);
    Translation2d camToTargetTranslation = new Translation2d(ctot.getX(), ctot.getY());

    // convert robot position to field zero coords from known target position
    Translation2d camToFieldTranslation = target_trans.minus(camToTargetTranslation);
    // robot position determined from tag and current gyro angle to create robot
    // pose
    Pose2d fieldToRobot = new Pose2d(camToFieldTranslation, gyroAngle);

    // TODO: compensate for camera position on robot

    return fieldToRobot;
  }

  @Override
  public void run() {
    cam.start();
    SmartDashboard.putString("Tag", "no valid tags visible");

    while (!Thread.interrupted()) {
      try {
        Thread.sleep(50);
        long endtime = 0;

        long startTime = System.nanoTime();
        Mat mat = cam.getFrame();
        endtime = System.nanoTime();
        double duration=(endtime - startTime);
        double total=duration;
        grab_time+=duration;

        startTime = System.nanoTime();
        AprilTag[] tags = null;
        if (m_drivetrain.useTags())
          tags = getTags(mat);
        endtime = System.nanoTime();
        duration=(endtime - startTime);
        total+=duration;

        detect_time +=  duration;
        count++;
        if (count > 1)
          maxtime = total > maxtime ? total : maxtime;
        String s = String.format("max:%-3.1f ave grab:%-2.1f detect:%-2.1f ms", 1e-6*maxtime, 1e-6*grab_time / count, 1e-6*detect_time/count);
        SmartDashboard.putString("Detect", s);
        if (!TargetMgr.tagsPresent() || !m_drivetrain.useTags()) {
          ouputStream.putFrame(mat);
          continue;
        }

        target_tag = null;
        pose_err = 4;
        double min_err = 1e6;
        for (int i = 0; i < tags.length; i++) {
          AprilTag tag = tags[i];

          double err = tag.getPoseError();

          if (target_id == BEST) {
            if (err <= min_err) {
              target_tag = tag;
              min_err = err;
              pose_err = err;
            }
          } else {
            if (tag.getTagId() == target_id) {
              target_tag = tag;
              pose_err = err;
              break;
            }
          }
        }
        last_pose = null;
        if (target_tag != null) {
          last_pose = getRobotPoseFromTag(target_tag, m_drivetrain.gyroRotation2d());
          if (last_pose != null) { // could be an incorrect tag identifier (index out of bounds for expected tag_id
                                   // range)
            Translation2d show = last_pose.getTranslation();
            if (TargetMgr.fieldRelative())
              show = TargetMgr.robotToFRC(camToTarget(target_tag, m_drivetrain.gyroRotation2d()),
                  target_tag.getTagId());
            String str = String.format("id:%d D:%-2.2f P:%-2.1f H:%-2.1f Robot: X:%-2.1f Y:%-2.1f",
                target_tag.getTagId(), target_tag.getDistance(), target_tag.getPitch(), target_tag.getYaw(),
                show.getX(), show.getY());
            SmartDashboard.putString("Tag", str);
          }
        } else {
          SmartDashboard.putString("Tag", "no valid tags visible");
          // m_drivetrain.setVisionPose(null);
        }

        for (int i = 0; i < tags.length; i++) {
          if (target_id != BEST && i != target_id)
            continue;
          AprilTag tag = tags[i];
          if (first)
            tag.print();

          Point c = tag.center();

          Scalar lns = new Scalar(255.0, 255.0, 0.0);
          Imgproc.line(mat, tag.tl(), tag.tr(), lns, 2);
          Imgproc.line(mat, tag.tr(), tag.br(), lns, 2);
          Imgproc.line(mat, tag.br(), tag.bl(), lns, 2);
          Imgproc.line(mat, tag.bl(), tag.tl(), lns, 2);

          // Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 0.0), 2);
          Imgproc.drawMarker(mat, c, new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 35, 2, 8);
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
        }
        first = false;

        ouputStream.putFrame(mat);
      } catch (Exception ex) {
        System.out.println("exception:" + ex);
      }
    }
  }
}
