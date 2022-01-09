// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cameraserver/CameraServer.h>

#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

int main(int _argc, char **_argv) {
  cs::CvSource outputStream("Blur", cs::VideoMode::PixelFormat::kMJPEG, 640,
                            480, 30);
  cs::MjpegServer mjpegServer2("test", 9000);
  mjpegServer2.SetSource(outputStream);
  cv::Mat mat = cv::imread("test.jpg", cv::ImreadModes::IMREAD_COLOR);
  int cnt = 0;
  while (true) {
    try {
      std::cout << "publishing image " << cnt++ << std::endl;
      outputStream.PutFrame(mat);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } catch (const std::exception &e) {
      std::cout << "Exception quiting" << std::endl;
      break;
    }
  }
}