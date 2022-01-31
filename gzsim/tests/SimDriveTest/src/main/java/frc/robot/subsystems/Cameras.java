// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import frc.robot.objects.Camera;
import frc.robot.objects.CameraInterface;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cameras extends SubsystemBase {

  public static final int FRONT_CAMERA = 1;
	public static final int BACK_CAMERA = 2;

  private Camera front_camera;
	private Camera back_camera;
	private CameraInterface recording_camera;

  /** Creates a new Cameras. */
  private static  ArrayList<CameraInterface> m_cameras= new  ArrayList<CameraInterface>();
 
  public Cameras(){
    SmartDashboard.putBoolean("record", false);
		SmartDashboard.putBoolean("front", true);

    front_camera=new Camera(FRONT_CAMERA);
		addCamera(front_camera);

		back_camera=new Camera(BACK_CAMERA);
		addCamera(back_camera);
  }

  public static void addCamera(CameraInterface cam){
    m_cameras.add(cam);
  }
  public static CameraInterface getCamera(int n) {
    for (int i = 0; i < m_cameras.size(); i++) {
      CameraInterface c = m_cameras.get(i);
      if (c.getChannel() == n) {
        return c;
      }
    }
    return null;
  }
  @Override
  public void periodic() {
    int which_camera=SmartDashboard.getBoolean("front", true)?FRONT_CAMERA:BACK_CAMERA;
		if(SmartDashboard.getBoolean("record", false)){
			recording_camera=Cameras.getCamera(which_camera);
			if(recording_camera!=null)
				recording_camera.record();
		}
		else if(recording_camera!=null)
			recording_camera.stop();
    // This method will be called once per scheduler run
  }
}
