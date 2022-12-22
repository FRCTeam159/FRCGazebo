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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.objects.Camera;
import utils.TagDetectorJNI;
import utils.TagResult;

public class AprilTagDetector extends Thread{
  static {
	  System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
  Camera camera;
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

  private final boolean time_detection = true;

  protected static CvSource ouputStream;
  protected TagDetectorJNI detector=new TagDetectorJNI(0);

  static String  test_image=System.getenv("GZ_SIM")+"/docs/apriltag_0_test.jpg";
  public AprilTagDetector() {
    camera=new Camera(0);
    detector.test(test_image,true,false);
    ouputStream = CameraServer.putVideo("testCamera", image_width, image_height);
    System.out.println(hFOV+" "+vFOV+" "+fx+" "+fy);
    test();
  }

  public void test(){
    Mat mat = Imgcodecs.imread(test_image);
    TagResult[] tags=detector.detect(mat,tw,fx,fy,cx,cy);
    for(int i=0;i<tags.length;i++){
      tags[i].print();
    }
  }
  @Override
  public void run() {
    boolean first=true;
    camera.start();
    while (true) {
      try {
        Thread.sleep(50);
        Mat mat = camera.getFrame();
        long startTime = System.nanoTime();
        TagResult[] tags=detector.detect(mat,tw,fx,fy,cx,cy);
        long endTime = System.nanoTime();

        double duration = (endTime - startTime)/1.0e6;  //divide by 1000000 to get milliseconds.
        if(time_detection){
          String s=String.format(" %2.1f ms",duration);
          SmartDashboard.putString("Detect", s);
        }

        //System.out.println("tag detect time="+duration);

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
        
        ouputStream.putFrame(mat);
        first=false;
      } catch (Exception ex) {
        System.out.println("exception:"+ex);
      }
    }
  }
}
