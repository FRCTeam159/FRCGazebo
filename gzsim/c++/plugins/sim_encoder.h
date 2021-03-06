// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>


/**
 * \brief Plugin for reading the speed and relative angle of a joint.
 *
 * This plugin publishes the angle since last reset and the speed of a
 * given joint to subtopics of the given topic every physics
 * update. There is also a control subtopic that takes three commands:
 * "start", "stop" and "reset":
 *
 * - "start": Start counting ticks from the current count.
 * - "stop":  Stop counting ticks, pauses updates.
 * - "reset": Set the current angle to zero.
 *
 * To add a encoder to your robot, add the following XML to your
 * robot model:
 *
 *     <plugin name="my_encoder" filename="sim_encoder{.so,.dll}">
 *       <joint>Joint Name</joint>
 *       <topic>~/my/topic</topic>
 *       <units>{degrees, radians}</units>
 *       <multiplier>Number</multiplier>
 *     </plugin>
 *
 * - `joint`: Name of the joint this encoder is attached to.
 * - `topic`: Used as the root for subtopics. `topic`/position
 *    The suggested value for topic is of the form
 *       ~/simulator/encoder/<n>
 *    where <n> is the number of the input channel
 *
 * - `units`: Optional. Defaults to radians.
 * - `multiplier`: Optional. Defaults to 1.
 *     This can be used to make the simulated encoder
 *     return a comparable number of ticks to a 'real' encoder
 *     Useful facts:  A 'degrees' encoder will report 360 ticks/revolution.
 *     The k4X encoder type can add another multiple of 4 into the mix.
 */
class Encoder : public gazebo::ModelPlugin {
 public:
  /// \brief Load the encoder and configures it according to the sdf.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  /// \brief Sends out the encoder reading each timestep.
  void Update(const gazebo::common::UpdateInfo& info);

 private:
  /// \brief Root topic for subtopics on this topic.
  std::string topic;

  /// \brief Whether or not this encoder measures radians or degrees.
  //bool radians;

  /// \brief A factor to multiply this output by.
  double multiplier;

  /// \brief Whether or not the encoder has been stopped.
  bool stopped;

  /// \brief The zero value of the encoder.
  double zero;

  /// \brief The value the encoder stopped counting at
  double stop_value;

  /// \brief The joint that this encoder measures
  gazebo::physics::JointPtr joint;

  /// \brief Callback for handling control data
  void Callback(const ConstGzStringPtr &msg);

  /// \brief Gets the current angle, taking into account whether to
  ///        return radians or degrees.
  double GetAngle();

  /// \brief Gets the current velocity, taking into account whether to
  ///        return radians/second or degrees/second.
  double GetVelocity();

  /// \brief The model to which this is attached.
  gazebo::physics::ModelPtr model;

  /// \brief Pointer to the world update function.
  gazebo::event::ConnectionPtr updateConn;

  /// \brief The node on which we're advertising.
  gazebo::transport::NodePtr node;

  /// \brief Subscriber handle.
  gazebo::transport::SubscriberPtr ctrl;

  /// \brief Publisher handles.
  gazebo::transport::PublisherPtr pub;
};

