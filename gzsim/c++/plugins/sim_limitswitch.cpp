// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma warning(push, 0)

#include "sim_limitswitch.h"

GZ_REGISTER_MODEL_PLUGIN(Switch)

void Switch::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

  if (model->GetJointCount() == 0) {
    std::cerr << "Invalid joint count, plugin not loaded\n";
    return;
  }
  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  std::string str = sdf->Get<std::string>("joint");
  this->joint = model->GetJoint(str);
  if (this->joint == NULL) {
    std::cerr << "could not find Joint element in plugin - reverting to "
                 "first joint\n";
    this->joint = model->GetJoints()[0];
  } 

  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  } else {
    topic = "~/" + sdf->GetAttribute("name")->GetAsString();
  }

  low = high = 0.0;
  low_limit=high_limit=false;

  if (sdf->HasElement("low")){
    low = sdf->Get<double>("low");
    low_limit=true;
  }
  if (sdf->HasElement("high")){
    high = sdf->Get<double>("high");
    high_limit=true;
  }

  std::cout << "Initializing sim_limitswitch: " << topic << " joint=" << joint->GetName()<< std::endl;
  
  // Connect to Gazebo transport for messaging

  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init("default");

  //std::cout << this->model->GetWorld()->Name() <<std::endl;

  ctrl = node->Subscribe(topic + "/control", &Switch::Callback, this);
  pub = node->Advertise<gazebo::msgs::Vector3d>(topic);

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Switch::Update, this, _1));
}

void Switch::Update(const gazebo::common::UpdateInfo& info) {
  gazebo::msgs::Vector3d msg;
  double l=0,h=0;
  double p = joint->Position(0);
  if(low_limit && p<=low)
    l=1;
  if(high_limit && p>=high)
    h=1;
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(l, h, p));
  pub->Publish(msg);
}

void Switch::Callback(ConstGzStringPtr  & msg) {
  std::string command = msg->data();
  std::cout << "Switch plugin received command:" << command << std::endl;
}


