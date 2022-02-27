#pragma once

#pragma warning(push, 0)

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <string>

/// \brief A plugin to control a Velodyne sensor.
class MotorPlugin : public gazebo::ModelPlugin {
  void OnMsg(ConstVector3dPtr &_msg);
  /// \brief A node used for transport
  gazebo::transport::NodePtr node;
  /// \brief A subscriber to a named topic.
  gazebo::transport::SubscriberPtr sub;
  /// \brief Pointer to the model.
  gazebo::physics::ModelPtr model;
  /// \brief Pointer to the joint.
  gazebo::physics::JointPtr joint;
  /// \brief A PID controller for the joint.
  gazebo::common::PID pid;
  double multiplier;
  bool debug;
  bool set_velocity;
  double last_vel;
  /// \brief Constructor
 public:
  MotorPlugin() {}
  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
 public:
  virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
  /// \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
  void SetVelocity(double vel);
};
