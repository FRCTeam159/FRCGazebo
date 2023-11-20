#pragma once

#pragma warning(push, 0)

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <string>

/// \brief A plugin to control a Velodyne sensor.
class Switch : public gazebo::ModelPlugin {
public:
/// \brief Load the encoder and configures it according to the sdf.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  /// \brief Sends out the encoder reading each timestep.
  void Update(const gazebo::common::UpdateInfo& info);

private:
  double low;
  double high;
  bool low_limit;
  bool high_limit;

  /// \brief Root topic for subtopics on this topic.
  std::string topic;

  /// \brief Callback for handling control data
  void Callback(const ConstGzStringPtr &msg);

  /// \brief The model to which this is attached.
  gazebo::physics::ModelPtr model;

/// \brief The joint that this encoder measures
  gazebo::physics::JointPtr joint;

  /// \brief Pointer to the world update function.
  gazebo::event::ConnectionPtr updateConn;

  /// \brief The node on which we're advertising.
  gazebo::transport::NodePtr node;

  /// \brief Subscriber handle.
  gazebo::transport::SubscriberPtr ctrl;

  /// \brief Publisher handles.
  gazebo::transport::PublisherPtr pub;
 
  /// \brief Constructor
 public:
  Switch() {}
 
};
