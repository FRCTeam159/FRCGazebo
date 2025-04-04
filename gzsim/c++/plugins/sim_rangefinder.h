/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once


#include <gazebo/gazebo.hh>

using namespace gazebo;

/**
 * \brief Plugin for reading the range of obstacles.
 *
 * This plugin publishes the range of obstacles detected by a sonar
 * rangefinder every physics update.
 *
 * To add a rangefinder to your robot, add the following XML to your
 * robot model:
 *
 *     <plugin name="my_rangefinder" filename="sim_rangefinder(.so,.dll)">
 *       <sensor>Sensor Name</sensor>
 *       <topic>~/my/topic</topic>
 *     </plugin>
 *
 * - `sensor`: Name of the sonar sensor that this rangefinder uses.
 * - `topic`: Optional. Message will be published as a gazebo::msgs::Vector3d.
 */
class Rangefinder: public ModelPlugin {
public:
  
  /// \brief Load the rangefinder and configures it according to the sdf.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

  /// \brief Sends out the rangefinder reading each timestep.
  void Update(const common::UpdateInfo &info);

  /// \brief Callback for handling control data
  void Callback(const ConstGzStringPtr &msg);

private:
  /// \brief Publish the range on this topic.
  std::string topic;

  /// \brief The sonar sensor that this rangefinder uses
  sensors::SonarSensorPtr sensor;

  physics::ModelPtr model;         ///< \brief The model that this is attached to.
  event::ConnectionPtr updateConn; ///< \brief Pointer to the world update function.
  transport::NodePtr node;         ///< \brief The node we're advertising on.
  transport::PublisherPtr pub;     ///< \brief Publisher handle.
  transport::SubscriberPtr ctrl;
};
