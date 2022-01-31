// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma warning(push, 0)

#include "sim_gyro.h"

GZ_REGISTER_MODEL_PLUGIN(Gyro)

void Gyro::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  this->model = model;

  // Parse SDF properties
  link = model->GetLink(sdf->Get<std::string>("link"));
  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  } else {
    topic = "~/" + sdf->GetAttribute("name")->GetAsString();
  }

  std::string axisString = sdf->Get<std::string>("axis");
  if (axisString == "roll") axis = Roll;
  if (axisString == "pitch") axis = Pitch;
  if (axisString == "yaw") axis = Yaw;

  //zero = GetAngle();
  stopped = false;

  std::cout << "Initializing gyro: " << topic << " link=" << link->GetName()
            << " axis=" << axis << std::endl;

  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init(this->model->GetWorld()->Name());

  sub = node->Subscribe(topic + "/control", &Gyro::Callback, this);
  pub = node->Advertise<gazebo::msgs::Vector3d>(topic);

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Gyro::Update, this, _1));
}

void Gyro::Update(const gazebo::common::UpdateInfo& info) {
  gazebo::msgs::Vector3d msg;
  double p=0,v=0;
 
    p = GetAngle();
    v = GetVelocity();

  gazebo::msgs::Set(&msg, ignition::math::Vector3d(p, v, 0));
  pub->Publish(msg);
}

void Gyro::Callback(ConstGzStringPtr& msg) {
  std::string command = msg->data();
  std::cout << "Gyro plugin received command:" << command << std::endl;
  /*
  if (command == "reset") {
    zero = GetAngle();
    stopped = false;
  } else if (command == "start") {
    stopped = false;
    zero = GetAngle();
  } else if (command == "stop") {
    stopped = true;
    stop_value = GetAngle()-zero;
  }else {
    gzerr << "WARNING: Gyro got unknown command '" << command << "'."
          << std::endl;
  }
  */
}

double Gyro::GetAngle() {
  return Limit(link->WorldCoGPose().Rot().Euler()[axis] * (180.0 / M_PI));
}

double Gyro::GetVelocity() {
  return link->RelativeAngularVel()[axis] * (180.0 / M_PI);
}

double Gyro::Limit(double value) {
  while (true) {
    if (value < -180)
      value += 360;
    else if (value > 180)
      value -= 360;
    else
      break;
  }
  return value;
}
