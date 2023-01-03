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

import edu.wpi.first.apriltag.jni.AprilTagJNI;
import edu.wpi.first.apriltag.jni.DetectionResult;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.objects.Camera;
import utils.TagDetectorJNI;
import utils.TagResult;
import frc.robot.objects.AprilTag;

public class AprilTagDetector extends Thread{
  static {
	  System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
  Camera cam;
  
  // use wpilib's tag detector or custom jni detector
  public static boolean use_wpi_detector=false;
  private final boolean time_detection = true;

  public static double maxPoseError=2;

  protected static CvSource ouputStream;
  protected TagDetectorJNI jni_detector;

  protected long wpi_detector=0;

  DriveTrain m_drivetrain;

  static double pose_err=2;
  static AprilTag target_tag=null;
  static public int BEST=-1;

  boolean first=true;

  static Pose2d last_pose=null;
  static int target_id=BEST;

  static String  test_image=System.getenv("GZ_SIM")+"/docs/apriltag_0_test.jpg";
  public AprilTagDetector(DriveTrain drivetrain) {
    m_drivetrain=drivetrain;
    makeDetector();

    cam=new Camera(0,640,480,40); // specs for Gazebo camera
    ouputStream = CameraServer.putVideo("testCamera", cam.image_width, cam.image_height);
    test();
  }

  // test tag detection jni using an image file
  public void test(){
    Mat mat = Imgcodecs.imread(test_image);
    //TagResult[] tags=detector.detect(mat,TargetMgr.targetSize,cam.fx,cam.fy,cam.cx,cam.cy);
    AprilTag[] tags=getTags(mat);
    for(int i=0;i<tags.length;i++){
      tags[i].print();
    }
  }
  
  void makeDetector(){
    if(use_wpi_detector)
      wpi_detector=AprilTagJNI.aprilTagCreate("tag16h5", 2.0, 0.0, 1, false, true);
    else
      jni_detector=new TagDetectorJNI(0);
  }
  AprilTag[] getTags(Mat mat){
    AprilTag[] atags;
    if(use_wpi_detector){
      Mat graymat=new Mat();
      Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
      DetectionResult[] detections=AprilTagJNI.aprilTagDetect(wpi_detector,
        graymat,true, 
        TargetMgr.targetSize,cam.fx,cam.fy,cam.cx,cam.cy, 1);
      int num_tags=detections.length;
      atags=new AprilTag[num_tags];
      for(int i=0;i<num_tags;i++)
        atags[i]=new AprilTag(detections[i]);
    }
    else{
      TagResult[] tags=jni_detector.detect(mat,
        TargetMgr.targetSize,cam.fx,cam.fy,cam.cx,cam.cy);
        int num_tags=tags.length;
        atags=new AprilTag[num_tags];
        for(int i=0;i<num_tags;i++)
          atags[i]=new AprilTag(tags[i]);
    }
    return atags;
  }
  
  static public void setTargetId(int i){
    target_id=i;
  }
  static public void setBestTarget(){
    target_id=BEST;
  }
  static public Pose2d getLastPose(){
    return last_pose;
  }
  // return pose ambiquity
  public static double  getPoseError() {
    return pose_err;
  }
  // project a scaler distance and angle to x and y coordinates
  public static Translation2d project(
			double radius, Rotation2d angle) {
		return new Translation2d(angle.getCos() * radius, angle.getSin() * radius);
	}

  // return position of tag relative to the camera
  public static Pose3d getTargetPose() {
   if(target_tag==null)
    return null;
   return  target_tag.getPose();
  }
  
  // Calculate the position of a robot on the field given a visible tag
  public static Pose2d getRobotPoseFromTag(AprilTag tag, Rotation2d gyroAngle) {
		
		TargetMgr.TagTarget target = TargetMgr.getTarget(tag.getTagId());
		if(target==null){ // sometime bad tag id is returned (e.g. 13 vs 0)
			System.out.println("null target:"+tag.getTagId());
			return null;
		}
		Pose3d pose = tag.getPose();

    double distance=tag.getDistance();
		Translation2d target_trans=target.getPose().getTranslation().toTranslation2d();
		
    // convert x,y, coords in camera space to camera angle
		double cam_angle=Math.atan2(pose.getTranslation().getY(), pose.getTranslation().getX());

	  // robot pose angle on field = gyro angle - camera angle
		Rotation2d camrot=new Rotation2d(cam_angle);
		Rotation2d rot=gyroAngle.minus(camrot);  
    // project camera posion to target onto the field
		Translation2d camToTargetTranslation=project(distance,rot);
		// convert robot position to field zero coords from known target position
		Translation2d camToFieldTranslation=target_trans.minus(camToTargetTranslation);
    // use robot position determined from tag and current gyro angle to create robot pose 
		Pose2d fieldToRobot=new Pose2d(camToFieldTranslation,gyroAngle);

		// TODO: compensate for camera position on robot
		
		return fieldToRobot;
	}

  @Override
  public void run() {
    cam.start();
    SmartDashboard.putString("Tag", "no valid tags visible");
    double maxtime=0;
    double avetime=0;
    int count=0;
    while (true) {
      try {
        Thread.sleep(20);
        long startTime = System.nanoTime();
        Mat mat = cam.getFrame();
        
        //TagResult[] tags=detector.detect(mat,TargetMgr.targetSize,cam.fx,cam.fy,cam.cx,cam.cy);
        AprilTag[] tags=getTags(mat);

        long endTime = System.nanoTime();
        if(time_detection){
          double duration = (endTime - startTime)/1.0e6;  //divide by 1000000 to get milliseconds.
          avetime+=duration;
          count++;
          if(count>1)
            maxtime=duration>maxtime?duration:maxtime;               
          String s=String.format("%-3.1f max:%-3.1f ave:%-2.1f ms",duration,maxtime,avetime/count);
          SmartDashboard.putString("Detect", s);
        }
        target_tag=null;
        pose_err=2;
        double max_err=2;
        for(int i=0;i<tags.length;i++){
          AprilTag tag=tags[i];
          double err=tag.getPoseError();

          if(target_id==BEST){
            if(err<max_err){
              target_tag=tag;
              max_err=err;
              pose_err=err;
            }
          }
          else{
            if(tag.getTagId()==target_id){
              target_tag=tag;
              pose_err=err;
              break;
            }
          }
        }
        last_pose=null;
        if(target_tag !=null){
          last_pose=getRobotPoseFromTag(target_tag,m_drivetrain.gyroRotation2d());
          if(last_pose !=null){ // could be an incorrect tag identifier (index out of bounds for expected tag_id range)
            String str = String.format("id:%d X:%-2.1f Y:%-2.1f H:%-2.1f P:%-2.1f D:%-2.2f E:%-2.2f",
               target_tag.getTagId(), last_pose.getX(), last_pose.getY(), target_tag.getYaw(),target_tag.getPitch(),target_tag.getDistance(),pose_err);
            //String str=best_tag.toString();
            SmartDashboard.putString("Tag", str);
          }
          //m_drivetrain.setVisionPose(pose);
        }
        else {
          SmartDashboard.putString("Tag", "no valid tags visible");
          //m_drivetrain.setVisionPose(null);
        }
       
        for(int i=0;i<tags.length;i++){
            if(target_id !=BEST && i !=target_id)
              continue;
            AprilTag tag=tags[i];
            if(first)
              tag.print();
          
            Point c= tag.center();
           
            Scalar lns=new Scalar(255.0, 255.0, 0.0);
            Imgproc.line(mat,tag.tl(),tag.tr(),lns,2);
            Imgproc.line(mat,tag.tr(),tag.br(),lns,2);
            Imgproc.line(mat,tag.br(),tag.bl(),lns,2);
            Imgproc.line(mat,tag.bl(),tag.tl(),lns,2);
           
            //Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 0.0), 2);
            Imgproc.drawMarker(mat, c, new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 35, 2, 8);
            Point p=new Point(tag.bl().x-10,tag.bl().y-10);
            Imgproc.putText (
              mat,                             // Matrix obj of the image
              "["+tag.getTagId()+"]",          // Text to be added
              p,               // point
              Imgproc.FONT_HERSHEY_SIMPLEX,    // front face
              1,                     // front scale
              new Scalar(255, 0, 0), // Scalar object for color
              2                      // Thickness
            );
        }
        first=false;
        
        ouputStream.putFrame(mat);
      } catch (Exception ex) {
        System.out.println("exception:"+ex);
      }
    }
  }
}
