// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma warning(push , 0)

#include "sim_clock.h"


GZ_REGISTER_MODEL_PLUGIN(Clock)

void Clock::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

  clock_time=zero_time=0;
  enabled=false;

  // Parse SDF properties
  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  } else {
    topic = "~/" + sdf->GetAttribute("name")->GetAsString();
  }

  std::cout << "Initializing clock: " << topic << std::endl;

  // Connect to Gazebo transport for messaging
  
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init(this->model->GetWorld()->Name());
  pub = node->Advertise<gazebo::msgs::Vector3d>(topic);
  ctrl = node->Subscribe(topic + "/control", &Clock::Callback, this);

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Clock::Update, this, _1));
}

void Clock::Update(const gazebo::common::UpdateInfo& info) {
  gazebo::msgs::Vector3d msg;
  double t=info.simTime.Double();
  if(!enabled) 
    zero_time=t;

  clock_time=t-zero_time;
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(clock_time, 0, 0));
  pub->Publish(msg);
}
void Clock::Callback(ConstGzStringPtr  & msg) {
  std::string command = msg->data();
  std::cout << "Clock plugin received command:" << command << std::endl;
  if (command == "reset") {
    zero_time=0;
    this->model->GetWorld()->ResetTime();
    this->model->GetWorld()->Reset();
  } 
  else if (command == "stop") {
    enabled =false;
  } 
  else if (command == "start") {
    enabled =true;
  } 
}
