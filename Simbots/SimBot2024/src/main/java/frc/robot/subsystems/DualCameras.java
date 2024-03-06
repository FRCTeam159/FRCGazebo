// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;

public class DualCameras extends Thread {
  public static boolean kTagCamera = true;

  static CvSource ouputStream;
  public static int kCamWidth = 320;
  public static int kCamHeight = 240;

  /** Creates a new Cameras. */
  public DualCameras() {
    ouputStream = CameraServer.putVideo("SwitchedCamera", kCamWidth, kCamHeight);
  }

  public void run() {
    while (!Thread.interrupted()) {
      try {
        Thread.sleep(50);
        Mat mat1 = TagDetector.getFrame();
        Mat mat2 = NoteDetector.getFrame();
        Mat mat;
         if (kTagCamera)
          mat=mat1;
         else
          mat=mat2;
        if(mat !=null)
          ouputStream.putFrame(mat);
      } catch (Exception ex) {
        System.out.println("DualCameras exception:" + ex);
      }
    }
  }

  public static void setTagCamera() {
    kTagCamera = true;
  }

  public static void setNoteCamera() {
    kTagCamera = false;
  }
}
