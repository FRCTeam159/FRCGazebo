#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin
  {

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: 
    virtual void Update();

    /// \brief Root topic for subtopics on this topic.
    std::string topic;
    
    sensors::ContactSensorPtr parentSensor;
    bool contact_made;

     /// \brief The node on which we're advertising.
    gazebo::transport::NodePtr node;

    /// \brief Subscriber handle.
    gazebo::transport::SubscriberPtr ctrl;

    /// \brief Publisher handles.
    gazebo::transport::PublisherPtr pub;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConn;
  };
}
#endif