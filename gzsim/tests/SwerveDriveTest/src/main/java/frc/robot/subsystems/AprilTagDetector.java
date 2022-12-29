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
import utils.TagTarget;
import utils.SimTargetMgr;


public class AprilTagDetector extends Thread{
  static {
	  System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
  Camera cam;
  
  private final boolean time_detection = true;

  public static double maxPoseError=6e-5;

  protected static CvSource ouputStream;
  protected TagDetectorJNI detector=new TagDetectorJNI(0);

  DriveTrain m_drivetrain;

  double best_err=1e6;
  TagResult best_tag=null;

  boolean first=true;

  static String  test_image=System.getenv("GZ_SIM")+"/docs/apriltag_0_test.jpg";
  public AprilTagDetector(DriveTrain drivetrain) {
    m_drivetrain=drivetrain;
    cam=new Camera(0,640,480,40); // specs for Gazebo camera
    //detector.test(test_image,true,false);
    ouputStream = CameraServer.putVideo("testCamera", cam.image_width, cam.image_height);
    test();
  }

  // test tag detection jni using an image file
  public void test(){
    Mat mat = Imgcodecs.imread(test_image);
    TagResult[] tags=detector.detect(mat,TagTarget.targetSize,cam.fx,cam.fy,cam.cx,cam.cy);
    for(int i=0;i<tags.length;i++){
      tags[i].print();
    }
  }

  // project a scaler distance and angle to x and y coordinates
  public static Translation2d project(
			double radius, Rotation2d angle) {
		return new Translation2d(angle.getCos() * radius, angle.getSin() * radius);
	}

  // return position of tag relative to the camera
  public Pose3d getTargetPose() {
   if(best_tag==null)
    return null;
   return  best_tag.getPose();
  }
  // Calculate the position of a robot on the field given a visible tag
  public static Pose2d getRobotPoseFromTag(TagResult tag, Rotation2d gyroAngle) {
		
		TagTarget target = SimTargetMgr.getTarget(tag.getTagId());
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

    while (true) {
      try {
        Thread.sleep(20);
        Mat mat = cam.getFrame();
        long startTime = System.nanoTime();
        TagResult[] tags=detector.detect(mat,TagTarget.targetSize,cam.fx,cam.fy,cam.cx,cam.cy);
        long endTime = System.nanoTime();

        double duration = (endTime - startTime)/1.0e6;  //divide by 1000000 to get milliseconds.
        if(time_detection){
          String s=String.format("%2.1f ms",duration);
          SmartDashboard.putString("Detect", s);
        }
        best_tag=null;
        best_err=1e6;
        for(int i=0;i<tags.length;i++){
          TagResult tag=tags[i];
          double err=tag.getPoseError();
          if(err<best_err){
            best_tag=tag;
            best_err=err;
          }
        }
        
        if(best_tag !=null && best_tag.getPoseError()<maxPoseError){
          String s=best_tag.toString();
          SmartDashboard.putString("Tag", s);
          m_drivetrain.setVisionPose(getRobotPoseFromTag(best_tag,m_drivetrain.gyroRotation2d()));
        }
        else {
          SmartDashboard.putString("Tag", "no valid tags visible");
          m_drivetrain.setVisionPose(null);
        }
       
        for(int i=0;i<tags.length;i++){
            TagResult tag=tags[i];
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
