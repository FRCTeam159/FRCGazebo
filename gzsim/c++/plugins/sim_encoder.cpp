// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma warning(push, 0)

#include "sim_encoder.h"

GZ_REGISTER_MODEL_PLUGIN(Encoder)

#define ROTATIONS 0.5/M_PI;

void Encoder::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
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

  multiplier = 1.0;
  zero = GetAngle();
  stopped = false;
  stop_value = 0;

  if (sdf->HasElement("multiplier"))
    multiplier = sdf->Get<double>("multiplier");

  std::cout << "Initializing sim_encoder: " << topic << " joint=" << joint->GetName()<< multiplier << std::endl;
  
  // Connect to Gazebo transport for messaging

  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init("default");

  //std::cout << this->model->GetWorld()->Name() <<std::endl;

  ctrl = node->Subscribe(topic + "/control", &Encoder::Callback, this);
  pub = node->Advertise<gazebo::msgs::Vector3d>(topic);

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Encoder::Update, this, _1));
}

void Encoder::Update(const gazebo::common::UpdateInfo& info) {
  gazebo::msgs::Vector3d msg;
  double p=0,v=0;
  if (stopped){
    p=stop_value * multiplier;
    zero = GetAngle();
  }
  else {
    p=(GetAngle() - zero) * multiplier;
    v= GetVelocity() * multiplier;
  }
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(p, v, stopped));
  pub->Publish(msg);
}

void Encoder::Callback(ConstGzStringPtr  & msg) {
  std::string command = msg->data();
  std::cout << "Encoder plugin received command:" << command << std::endl;
  if (command == "reset") {
    zero = GetAngle();
    stopped=false;
  } else if (command == "start") {
    stopped = false;
    zero = GetAngle();
  } else if (command == "stop") {
    stopped = true;
    stop_value = GetAngle();
  } else {
    gzerr << "WARNING: Encoder got unknown command '" << command << "'."
          << std::endl;
  }
}

double Encoder::GetAngle() {
  return joint->Position(0)*ROTATIONS;
}

double Encoder::GetVelocity() {
  return joint->GetVelocity(0)*ROTATIONS;
}
