// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

/**
 * \brief Plugin for publishing the simulation time.
 *
 * This plugin publishes the simualtaion time in seconds every physics
 * update.
 *
 * To add a clock to your robot, add the following XML to your robot
 * model:
 *
 *     <plugin name="my_clock" filename="libclock.so">
 *       <topic>~/my/topic</topic>
 *     </plugin>
 *
 * - `topic`: Optional. Message will be published as a gazebo.msgs.Float64.
 *
 * \todo Make WorldPlugin?
 */
class Clock : public gazebo::ModelPlugin {
 public:
  /// \brief Load the clock and configures it according to the sdf.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  /// \brief Sends out time each timestep.
  void Update(const gazebo::common::UpdateInfo& info);

  /// \brief Callback for handling control data
  void Callback(const ConstGzStringPtr &msg);

 private:
  double zero_time;
  double clock_time;
  bool enabled;
  /// \brief Publish the time on this topic.
  std::string topic;

  /// \brief The model to which this is attached.
  gazebo::physics::ModelPtr model;

  /// \brief Pointer to the world update function.
  gazebo::event::ConnectionPtr updateConn;

  /// \brief The node on which we're advertising.
  gazebo::transport::NodePtr node;

/// \brief Subscriber handle.
  gazebo::transport::SubscriberPtr ctrl;
  
  /// \brief Publisher handle.
  gazebo::transport::PublisherPtr pub;
};

