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
stream("sim-frames", cs::VideoMode::PixelFormat::kMJPEG, 320,240, 10),
server("sim-camera", 9000)
{
  FreeImage_Initialise();
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
/*
  std::cout << "FreeImageVersion:" << FreeImage_GetVersion() << std::endl;
  int ogre_verion = ((1 << 16) | (9 << 8) | 0);
  std::cout << "OgreVersion:" << OGRE_VERSION << " < " << ogre_verion
            << std::endl;
*/
  if (!this->parentSensor) {
    gzerr << "CameraPlugin not attached to a camera sensor\n";
    return;
  }
  this->camera = this->parentSensor->Camera();

  if (sdf->HasElement("save-path"))
    path = sdf->Get<std::string>("save-path");
  else
    path = "~/tmp";
  if (sdf->HasElement("enabled"))
    enabled = sdf->Get<bool>("enabled");
  else
    enabled = true;
  if (sdf->HasElement("save-frames"))
    saveMax = sdf->Get<int>("save-frames");
  else
    saveMax = 0;
  if (sdf->HasElement("fps"))
    fps = sdf->Get<int>("fps");
  else
    fps = 10;

  gzmsg << "Initializing sim_camera enabled ?" << enabled << " fps:" << fps<<"\n";

  saveCount = 0;

  width = this->camera->ImageWidth();
  height = this->camera->ImageHeight();
  depth = this->camera->ImageDepth();
  format = this->camera->ImageFormat();

  stream.SetVideoMode(cs::VideoMode::PixelFormat::kMJPEG, width, height,fps);
  server.SetSource(stream);

  bitmap = FreeImage_Allocate(width, height, 24);

  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      std::bind(&CameraPlugin::OnNewFrame, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);
}

//#define DEBUG
/////////////////////////////////////////////////
void CameraPlugin::OnNewFrame(const unsigned char *_image, unsigned int _width,
                              unsigned int _height, unsigned int _depth,
                              const std::string &_format) {
  
if(!enabled)
  return;
if (saveCount <= saveMax) {
  // The Windows Orge version being used only has support for png so ..
  //   this->parentSensor->Camera()->SaveFrame(_image, _width, _height,
  //    _depth,_format, tmp); 
  // generates a fatal error because Camera()->SaveFrame tries to save the image as a 
  // jpeg file 
  // As a work-aroung we can use FreeImage to output as jpeg

    char tmp[1024];
    snprintf(tmp, sizeof(tmp), "%s/frame_%04d.jpg", path.c_str(),
             this->saveCount);
    memcpy(FreeImage_GetBits(bitmap), _image, _width * _height * 3);
    FreeImage_FlipVertical(bitmap);
    FreeImage_Save(FIF_JPEG, bitmap, tmp);
    saveCount++;
  }
  // Instead of using mjpeg-streamer (which isn't supported on Windows) 
  // we can use WPI's MjpegServer and CvSource classes to avoid having to write out
  // jpeg files at all
  cv::Mat img(height, width, CV_8UC3, (void*)_image,3*width);
  stream.PutFrame(img); // publish directly to port 9000
}
