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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.objects.Camera;
import utils.TagDetectorJNI;
import utils.TagResult;
import utils.TagTarget;
import utils.SimTargetMgr;

import org.photonvision.PhotonUtils;


public class AprilTagDetector extends Thread{
  static {
	  System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
  Camera cam;
  /* 
  public int image_width = 640;
  public int image_height = 480;

  // parametes for sim camera and 0.5 m targets
  public double tw=0.4;
  public double hFOV=40;
  public double aspect=((double)image_width)/image_height;
  public double vFOV=hFOV/aspect;
  double cx=image_width/2.0;
  double cy=image_height/2.0;
  double fx=cx/Math.tan(0.5*Math.toRadians(hFOV));
  double fy=cy/Math.tan(0.5*Math.toRadians(vFOV));
*/
  private final boolean time_detection = true;

  protected static CvSource ouputStream;
  protected TagDetectorJNI detector=new TagDetectorJNI(0);

   //System.out.println("tag detect time="+duration);
   Translation2d cameraToRobotOffset=new Translation2d(-0.2,0);
   Transform2d cameraToRobot=new Transform2d(cameraToRobotOffset,new Rotation2d());

  static String  test_image=System.getenv("GZ_SIM")+"/docs/apriltag_0_test.jpg";
  public AprilTagDetector() {
    SimTargetMgr.setPerimTargets();
    cam=new Camera(0,640,480,40); // specs for Gazebo camera
    //detector.test(test_image,true,false);
    ouputStream = CameraServer.putVideo("testCamera", cam.image_width, cam.image_height);
    //System.out.println(hFOV+" "+vFOV+" "+fx+" "+fy);
    //test();
  }

  public void test(){
    Mat mat = Imgcodecs.imread(test_image);
    TagResult[] tags=detector.detect(mat,TagTarget.targetSize,cam.fx,cam.fy,cam.cx,cam.cy);
    for(int i=0;i<tags.length;i++){
      tags[i].print();
    }
  }
  @Override
  public void run() {
    boolean first=true;
    cam.start();
    while (true) {
      try {
        Thread.sleep(50);
        Mat mat = cam.getFrame();
        long startTime = System.nanoTime();
        TagResult[] tags=detector.detect(mat,TagTarget.targetSize,cam.fx,cam.fy,cam.cx,cam.cy);
        long endTime = System.nanoTime();

        double duration = (endTime - startTime)/1.0e6;  //divide by 1000000 to get milliseconds.
        if(time_detection){
          String s=String.format(" %2.1f ms",duration);
          SmartDashboard.putString("Detect", s);
        }

        double heading=SmartDashboard.getNumber("H", 0.0);
        Rotation2d gyroAngle=new Rotation2d(Math.toRadians(-heading));
        double best_err=1e6;
        TagResult best_tag=null;
        for(int i=0;i<tags.length;i++){
          TagResult tag=tags[i];
          double err=tag.getPoseError();
          if(err<best_err){
            best_tag=tag;
            best_err=err;
          }
        }
        if (best_tag != null) {
          TagResult tag = best_tag;
          Pose3d pose = tag.getPose();

          TagTarget target = SimTargetMgr.getPerimTarget(tag.getTagId());

          Rotation2d poseYaw = pose.getRotation().toRotation2d().unaryMinus();
          Pose2d fieldToTarget = target.getPose().toPose2d();

          double distance = pose.getTranslation().toTranslation2d().getNorm();
          Translation2d camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(distance, poseYaw);

          Transform2d cameraToTarget = new Transform2d(camToTargetTranslation, gyroAngle);
          Pose2d fieldToRobot = PhotonUtils.estimateFieldToRobot(cameraToTarget, fieldToTarget, cameraToRobot);

          SmartDashboard.putString("BestTag ", String.format("id:%d x:%-6.2f y:%-6.2f err:%1.2f",
              tag.getTagId(), fieldToRobot.getX(), fieldToRobot.getY(), 1e5 * tag.getPoseError()));
        }
        for(int i=0;i<tags.length;i++){
            TagResult tag=tags[i];
          
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
        
        ouputStream.putFrame(mat);
        first=false;
      } catch (Exception ex) {
        System.out.println("exception:"+ex);
      }
    }
  }
}
