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
#ifndef GAZEBO_PLUGINS_CAMERAPLUGIN_HH_
#define GAZEBO_PLUGINS_CAMERAPLUGIN_HH_

#pragma once

#pragma warning(push, 0)
#include <windows.h>

#include <boost/asio.hpp>
#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo {
class GZ_PLUGIN_VISIBLE CameraPlugin : public SensorPlugin {
 public:
  CameraPlugin();

  /// \brief Destructor
  virtual ~CameraPlugin();

  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  virtual void OnNewFrame(const unsigned char *_image, unsigned int _width,
                          unsigned int _height, unsigned int _depth,
                          const std::string &_format);

 protected:
  unsigned int width, height, depth;
  std::string format;

  sensors::CameraSensorPtr parentSensor;
  rendering::CameraPtr camera;
  std::string path;
  bool enabled;

 private:
  event::ConnectionPtr newFrameConnection;
  int saveCount;
  int saveMax;
};
}  // namespace gazebo

#endif
