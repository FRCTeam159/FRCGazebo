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
  
  num_axis=3;
  axis=Yaw;
  
  if (sdf->HasElement("num_axis")){
    num_axis = sdf->Get<int>("num_axis");
    std::cout << "Initializing gyro: " << topic << " link=" << link->GetName()
            << " num_axis:"<<num_axis<< std::endl;
  }
  if(num_axis==1){
    std::string axisString = sdf->Get<std::string>("axis");
    if (axisString == "roll") axis = Roll;
    if (axisString == "pitch") axis = Pitch;
    if (axisString == "yaw") axis = Yaw;
    std::cout << "Initializing gyro: " << topic << " link=" << link->GetName()
            << " axis=" << axisString << std::endl;
  }

  //zero = GetAngle();
  stopped = false;


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
  double p1=0,p2=0,p3=0;
  if(num_axis==1){
    p1 = GetAngle(axis);
    ignition::math::Pose3d pose = link->WorldCoGPose();
    ignition::math::Vector3<double> position = pose.Pos();
    p2 =position.X();
    p3 =position.Y();
    //p3 = GetVelocity(axis); 
  }
  else{
    p1=GetAngle(Yaw);
    p2=GetAngle(Roll);
    p3=GetAngle(Pitch);
  }
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(p1, p2, p3));
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

double Gyro::GetAngle(ROTATION a) {
  return Limit(link->WorldCoGPose().Rot().Euler()[a] * (180.0 / M_PI));
}

double Gyro::GetVelocity(ROTATION a) {
  return link->RelativeAngularVel()[a] * (180.0 / M_PI);
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
