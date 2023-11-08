/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "sim_rangefinder.h"

// #include <gazebo/physics/physics.hh>
// #include <gazebo/transport/transport.hh>
 #include <gazebo/sensors/sensors.hh>
 #include <boost/pointer_cast.hpp>

// #include <boost/pointer_cast.hpp>

GZ_REGISTER_MODEL_PLUGIN(Rangefinder)

void Rangefinder::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

  // Parse SDF properties
  sensor = boost::dynamic_pointer_cast<sensors::SonarSensor>(
               sensors::get_sensor(sdf->Get<std::string>("sensor")));
  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  } else {
    topic = "~/"+sdf->GetAttribute("name")->GetAsString();
  }

  std::cout << "Initializing sim_rangefinder: " << topic << " sensor=" << sensor->Name() << std::endl;

  // Connect to Gazebo transport for messaging
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init("default");
  ctrl = node->Subscribe(topic + "/control", &Rangefinder::Callback, this);
  pub = node->Advertise<gazebo::msgs::Vector3d>(topic);

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = event::Events::ConnectWorldUpdateBegin(boost::bind(&Rangefinder::Update, this, _1));
}

void Rangefinder::Update(const common::UpdateInfo &info) {
  gazebo::msgs::Vector3d msg;
  double r=sensor->Range();
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(r, 0, 0));
  pub->Publish(msg);
}

void Rangefinder::Callback(ConstGzStringPtr  & msg) {
  std::string command = msg->data();
  std::cout << "Rangefinder plugin received command:" << command << std::endl;
}
