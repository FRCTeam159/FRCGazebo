// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Targeting;

public class CameraStreams extends Thread {
  TargetDetector m_front_detector;
  TargetDetector m_back_detector;
  boolean front_camera = true;

  protected static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected static final NetworkTable m_target_table=inst.getTable("TargetData");

  protected static CvSource dualStream;
  public int image_width = 640;
  public int image_height = 480;
  private NetworkTableEntry frontCam; // camera (0=front 1=back)
  private static Mat m1,m2;

  Targeting m_targeting;

  protected final Timer m_timer = new Timer();

  public CameraStreams(Targeting targeting) {
    m_targeting=targeting;
    System.out.println("new Targeting " + Robot.isReal());
    SmartDashboard.putBoolean("Front Camera", true);
    SmartDashboard.putBoolean("Show HVS threshold", false);
    frontCam= m_target_table.getEntry("frontCamera");

    dualStream = CameraServer.putVideo("SwitchedCamera", image_width, image_height);

    if (Robot.isReal()) {
      m_front_detector = new LimelightDetector();
      m_back_detector = new AxonDetector();
    } else {
      m_back_detector = new GripBallDetector();
      m_front_detector = new GripTapeDetector();
    }
  }

  void putFrame(CvSource source, Mat m) {
    if (m != null)
      source.putFrame(m);
  }

  @Override
  public void run() {
    m_timer.start();
    m_targeting.setFrontTarget(true);
    m_front_detector.setTargetSpecs();
    while (true) {
      try {
        Thread.sleep(20);
      } catch (InterruptedException ex) {
        System.out.println("exception)");
      }
      TargetDetector.show_hsv_threshold = SmartDashboard.getBoolean("Show HVS threshold", true);
      boolean is_front = m_targeting.frontCamera();
      //tc.setBoolean(is_front);
      if (is_front != front_camera) {
        if (is_front){
          System.out.println("CameraStreams setting front camera");
          m_front_detector.setTargetSpecs();
        }
        else{
          System.out.println("CameraStreams setting back camera");
          m_back_detector.setTargetSpecs();
        }
      }
      m1=m_front_detector.getFrame();
      m2=m_back_detector.getFrame();
      if (is_front && m1 !=null) {
        m_front_detector.process();
        putFrame(dualStream, m1);
        frontCam.setBoolean(true);
      } else if (m2 !=null){
        m_back_detector.process();
        putFrame(dualStream, m2);
        frontCam.setBoolean(false);
      }
      front_camera = is_front;
    }
  }
}
