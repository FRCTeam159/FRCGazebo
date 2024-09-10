// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;

public class Cameras extends Thread {
  static CvSource ouputStream;
  public static int kCamWidth = 320;
  public static int kCamHeight = 240;
  private final GazeboCamera m_gzcamera = new GazeboCamera(kCamWidth, kCamHeight);

  /** Creates a new Camera. */
  public Cameras() {
    ouputStream = CameraServer.putVideo("Camera", kCamWidth, kCamHeight);   
  }

 public void run() {
    while (!Thread.interrupted()) {
      try {
        Thread.sleep(50);     
        Mat mat= m_gzcamera.getFrame();   
        if(mat !=null)
          ouputStream.putFrame(mat);
      } catch (Exception ex) {
        System.out.println("DualCameras exception:" + ex);
      }
    }
  }
}
