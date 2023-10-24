// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//====================================================
// Problem with VideoCapture 
// vcap.read(mat) returns error "CAP_IMAGES: can't find starting number"
// source of problem: 
//  - missing opencv_videoio_ffmjpeg452_64.dll in path
//  - not included in wpilib Maven repo (3rd party / opencv)
// work-arounds:
// 1) use new class in utils (MJpegClient) instead (not based on ffmjpeg)
// 2) found opencv_videoio_ffmjpeg453_64.dll in conda pkgs folder
//     renamed it to opencv_videoio_ffmjpeg45_64.dll and placed in a path folder (bin)
//====================================================

package frc.robot.objects;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import utils.MJpegClient;

/** Add your docs here. */
public class MJpegReader implements VideoInterface {
    public String input_url;
    boolean connected = false;
    VideoCapture vcap;
    MJpegClient mjpeg_reader;
    private boolean use_vcap=false;

    MJpegReader(String url) {
        input_url=url;
        if (use_vcap) {
            vcap = new VideoCapture();
            connected = vcap.open(input_url);
        } else {
            mjpeg_reader = new MJpegClient(input_url);
            connected = mjpeg_reader.isConnected();
        }
        if (!connected) 
            System.out.println("Error opening video stream " + input_url);
        else 
            System.out.println("Video Stream captured " + input_url);
    }

    @Override
    public boolean isConnected() {
        return connected;
    }

    @Override
    public void process() {
    }
    
    @Override
    public Mat getFrame() {
        Mat mat = new Mat();
        if (use_vcap) {
            if (!vcap.read(mat))
                return null;
        } else {
            mat = mjpeg_reader.read();
            if (mat == null)
                return null;
        }
        return mat;
    }
}
