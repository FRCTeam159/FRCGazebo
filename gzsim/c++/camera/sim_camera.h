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
#include <boost/asio.hpp>
#include <windows.h>

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
#include "gazebo/sensors/DepthCameraSensor.hh"
#include <FreeImage.h>

#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace gazebo
{
  class GZ_PLUGIN_VISIBLE CameraPlugin : public SensorPlugin
  {
    public: CameraPlugin();

    /// \brief Destructor
    public: virtual ~CameraPlugin();

    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    protected: unsigned int width, height, depth;
    protected: std::string format;

    protected: double fps;

    protected: sensors::CameraSensorPtr parentSensor;
    protected: rendering::CameraPtr camera;
    protected: FIBITMAP *bitmap;

    private: event::ConnectionPtr newFrameConnection;
    private: int saveCount;
    private: int saveMax;
    private: int port;
    private: int video_index;
    protected: std::string savepath;
    protected: std::string video_file;
    protected: std::string topic;
    protected: bool record_video;
    protected: bool enabled;
    protected: bool stopped;
    protected: gazebo::transport::SubscriberPtr ctrl;
    protected: gazebo::transport::NodePtr node;
    protected: void Callback(const ConstGzStringPtr &msg);
    protected: cv::VideoWriter video;

    protected: cs::CvSource stream;
    protected: cs::MjpegServer server;

  };
}

#endif
