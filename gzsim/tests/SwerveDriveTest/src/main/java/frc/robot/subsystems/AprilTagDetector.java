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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.objects.Camera;
import utils.TagDetectorJNI;
import utils.TagResult;
import utils.TagTarget;

public class AprilTagDetector extends Thread{
  static {
	  System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
  Camera cam;
  
  private final boolean time_detection = true;

  protected static CvSource ouputStream;
  protected TagDetectorJNI detector=new TagDetectorJNI(0);

  DriveTrain m_drivetrain;

  static String  test_image=System.getenv("GZ_SIM")+"/docs/apriltag_0_test.jpg";
  public AprilTagDetector(DriveTrain drivetrain) {
    m_drivetrain=drivetrain;
    cam=new Camera(0,640,480,40); // specs for Gazebo camera
    //detector.test(test_image,true,false);
    ouputStream = CameraServer.putVideo("testCamera", cam.image_width, cam.image_height);
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
    cam.start();
    while (true) {
      try {
        Thread.sleep(20);
        Mat mat = cam.getFrame();
        long startTime = System.nanoTime();
        TagResult[] tags=detector.detect(mat,TagTarget.targetSize,cam.fx,cam.fy,cam.cx,cam.cy);
        long endTime = System.nanoTime();

        double duration = (endTime - startTime)/1.0e6;  //divide by 1000000 to get milliseconds.
        if(time_detection){
          String s=String.format(" %2.1f ms",duration);
          SmartDashboard.putString("Detect", s);
        }

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
       
        m_drivetrain.setTag(best_tag);
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
      } catch (Exception ex) {
        System.out.println("exception:"+ex);
      }
    }
  }
}
