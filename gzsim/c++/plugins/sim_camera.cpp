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
//#include <FreeImage.h>
#include <OgrePixelFormat.h>
#include "gazebo/rendering/ogre_gazebo.h"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)

/////////////////////////////////////////////////
CameraPlugin::CameraPlugin() : SensorPlugin(), width(0), height(0), depth(0) {
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

  std::cout<<"FreeImageVersion:"<<FreeImage_GetVersion()<<std::endl;
  int ogre_verion=((1 << 16) | (9 << 8) | 0);
  std::cout<<"OgreVersion:"<<OGRE_VERSION<<" < "<<ogre_verion<<std::endl;

  if (!this->parentSensor) {
    gzerr << "CameraPlugin not attached to a camera sensor\n";
    return;
  }
  this->camera = this->parentSensor->Camera();

  if (sdf->HasElement("path")) 
    path = sdf->Get<std::string>("path");
  else 
    path = "~/tmp";
  if (sdf->HasElement("enabled")) 
    enabled = sdf->Get<bool>("enabled");
  else
    enabled=true;
  if (sdf->HasElement("maxFrames")) 
    saveMax = sdf->Get<int>("maxFrames");
  else
    saveMax = 10;

  gzmsg << "Initializing sim_camera:" << enabled << " path:" << path << " saveMax:"<<saveMax<<"\n";

  saveCount=0;

  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->depth = this->camera->ImageDepth();
  this->format = this->camera->ImageFormat();

  bitmap=FreeImage_Allocate(width,height,24);

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
  
  char tmp[1024];
  //const char *c = path.c_str();
  snprintf(tmp, sizeof(tmp), "%s/frame_%04d.jpg", path.c_str(),this->saveCount);

if (saveCount > saveMax) 
    return;
#ifdef DEBUG
  gzmsg << "Saving frame [" << this->saveCount << "] as [" << tmp << "] format:"<<_format<<"\n";
#endif

 // This Orge version only supports png using this function
 // this->parentSensor->Camera()->SaveFrame(_image, _width, _height, _depth,_format, tmp);

  // instead use FreeImage to output as jpeg (for mjpeg-streamer)
  
  memcpy(FreeImage_GetBits(bitmap), _image, _width * _height * 3);
  FreeImage_FlipVertical(bitmap);
  FreeImage_Save(FIF_JPEG,bitmap,tmp);

  saveCount++;

}
