// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.opencv.core.Core;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
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

  protected static CvSource ouputStream;
  protected TagDetectorJNI detector=new TagDetectorJNI(0);

  public AprilTagDetector() {
    camera=new Camera(0);
    //detector.test(System.getenv("GZ_SIM")+"/docs/apriltag_test.jpg");
    ouputStream = CameraServer.putVideo("testCamera", image_width, image_height);
  }

  @Override
  public void run() {

    while (true) {
      try {
        Thread.sleep(50);
        Mat mat = camera.getFrame();

        //long startTime = System.nanoTime();
        TagResult[] tags=detector.detect(mat);
       //long endTime = System.nanoTime();

        //long duration = (endTime - startTime)/1000;  //divide by 1000000 to get milliseconds.
       // System.out.println("tag detect time="+duration);

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
            Point p=new Point(tag.bl().x-10,tag.bl().y-10);;
            Imgproc.putText (
              mat,                          // Matrix obj of the image
              "["+tag.getTagId()+"]",          // Text to be added
              p,               // point
              Imgproc.FONT_HERSHEY_SIMPLEX,      // front face
              1,                               // front scale
              new Scalar(255, 0, 0),             // Scalar object for color
              2                                // Thickness
            );
        }
        
        ouputStream.putFrame(mat);
      } catch (Exception ex) {
        System.out.println("exception:"+ex);
      }
    }
  }
}
