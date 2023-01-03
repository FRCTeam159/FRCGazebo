/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "sim_camera.h"


using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)

/////////////////////////////////////////////////
CameraPlugin::CameraPlugin() : SensorPlugin(), width(0), height(0), depth(0),
stream("sim-frames", cs::VideoMode::PixelFormat::kMJPEG, 640,480, 10)
{
  FreeImage_Initialise();
  stopped=true;
}

/////////////////////////////////////////////////
CameraPlugin::~CameraPlugin() {
  this->newFrameConnection.reset();
  this->parentSensor.reset();
  this->camera.reset();
}

/////////////////////////////////////////////////
void CameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr sdf) {
  if (!_sensor) gzerr << "Invalid sensor pointer.\n";

  this->parentSensor =
      std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor) {
    gzerr << "CameraPlugin not attached to a camera sensor\n";
    return;
  }
  this->camera = this->parentSensor->Camera();

  if (sdf->HasElement("savepath"))
    savepath = sdf->Get<std::string>("savepath");
  else
    savepath = "~/tmp";

  if (sdf->HasElement("enabled"))
    enabled = sdf->Get<bool>("enabled");
  else
    enabled = true;

  if (sdf->HasElement("maxframes"))
    saveMax = sdf->Get<int>("maxframes");
  else
    saveMax = 0;

  if (sdf->HasElement("fps"))
    fps = sdf->Get<int>("fps");
  else
    fps = 10;
  if (sdf->HasElement("port"))
    port = sdf->Get<int>("port");
  else
    port = 9000;

  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  } else {
    topic = "~/" + sdf->GetAttribute("name")->GetAsString();
  }

  server = cs::MjpegServer("sim-camera", port);

  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init("default");

  ctrl = node->Subscribe(topic + "/control", &CameraPlugin::Callback, this);

  saveCount = 0;

  width = this->camera->ImageWidth();
  height = this->camera->ImageHeight();
  depth = this->camera->ImageDepth();
  format = this->camera->ImageFormat();

  if (sdf->HasElement("video")){
    video_file = savepath+"/"+sdf->Get<std::string>("video");
    record_video=true;
  }
  else{
    video_file = "";
    record_video=false;
  }
  video_index=0;
  
  gzmsg << "Initializing sim_camera"<<" port="<<port<<" movie=" << record_video << " fps=" << fps<<"\n";

  stream.SetVideoMode(cs::VideoMode::PixelFormat::kMJPEG, width, height,fps);
  server.SetSource(stream);

  bitmap = FreeImage_Allocate(width, height, 24);

  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      std::bind(&CameraPlugin::OnNewFrame, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);
  
}

/////////////////////////////////////////////////
void CameraPlugin::OnNewFrame(const unsigned char *_image, unsigned int _width,
                              unsigned int _height, unsigned int _depth,
                              const std::string &_format) {
  
if(!enabled)
  return;
if (!record_video && saveMax && (saveCount < saveMax)) {  
  // note: save jpeg frames into a directory
  // - no longer needed for mjpeg streamer
  // - but could be used to capture frames for generating a video
  // note: The Windows Orge version being used by Gazebo only has support for png so the call: 
  //    this->parentSensor->Camera()->SaveFrame(_image, _width, _height, _depth,_format, tmp); 
  //    generates a fatal error because Camera()->SaveFrame tries to save the image as a 
  //    jpeg file 
  //    As a work-aroung we can use FreeImage to output as jpeg

    char tmp[1024];
    snprintf(tmp, sizeof(tmp), "%s/frame_%04d.jpg", savepath.c_str(),
             this->saveCount);
    memcpy(FreeImage_GetBits(bitmap), _image, _width * _height * 3);
    FreeImage_FlipVertical(bitmap);
    FreeImage_Save(FIF_JPEG, bitmap, tmp);
    std::cout << "saving image:" << saveCount << " "<<tmp<<std::endl;

    saveCount++;
  }
  // Instead of using mjpeg-streamer (which isn't supported on Windows) 
  // we can use WPI's MjpegServer and CvSource classes to avoid having to write out
  // jpeg files at all
  cv::Mat bgr(height, width, CV_8UC3, (void*)_image,3*width);
  cv::Mat rgb;
  // Apparently the Mat constructor creates a BGR image so need to convert it to RGB
  cv::cvtColor( bgr, rgb, cv::COLOR_BGR2RGB );
  stream.PutFrame(rgb); // publish directly to port
  if(!stopped && record_video  && video.isOpened())
    video.write(rgb);
}

void CameraPlugin::Callback(ConstGzStringPtr  & msg) {
  std::string command = msg->data();
  std::cout << "CameraPlugin received command:" << command << std::endl;
  if (command == "disable") {
    enabled=false;
  } 
  else if (command == "enable") {
    enabled=true;
  }
  else if (command == "reset") {
    this->saveCount=0; // should also clear the tmp directory if saving jpegs
    stopped=true;
    video_index=0;
    if(!stopped && record_video  && video.isOpened()){
      stopped=true;
      video.release();
    }
  } 
  else if (command == "run") {   // start recording
    if(record_video && stopped){
      if(video.isOpened())
        video.release();
      std::string ext="avi";
      int position=video_file.find_last_of(".");
      if(position != std::string::npos)
         ext = video_file.substr(position+1);
      std::string stem = video_file.substr(0,position);
     
      std::string path=stem+"_"+std::to_string(video_index)+"."+ext;
      video.open(path, cv::VideoWriter::fourcc('M','J','P','G'),fps, cv::Size(width,height));
      video_index++;
      stopped=false;
      std::cout <<"starting video capture:"<<path<<std::endl;
    }
    stopped = false;
  } else if (command == "stop") { // stop recording
    if(!stopped && record_video  && video.isOpened()){
      std::cout <<"ending video capture"<<std::endl;
      video.release();
      stopped = true;
    }
  } else {
    gzerr << "WARNING: Encoder got unknown command '" << command << "'."
          << std::endl;
  }
}
