/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "sim_piston.h"

#include <boost/algorithm/string/replace.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

GZ_REGISTER_MODEL_PLUGIN(PneumaticPiston)

void PneumaticPiston::Load(gazebo::physics::ModelPtr model,
                           sdf::ElementPtr sdf) {
  this->model = model;
  signal = 0;

  // Parse SDF properties
  std::string str = sdf->Get<std::string>("joint");

  this->joint = model->GetJoint(str);
  if (this->joint == NULL){
    std::cerr << "could not find Joint element in plugin - reverting to "
                 "first joint\n";
    this->joint = model->GetJoints()[0];
  }
  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  } else {
    topic = "~/" + sdf->GetAttribute("name")->GetAsString();
  }

  forward_force = sdf->Get<double>("forward-force");
  reverse_force = sdf->Get<double>("reverse-force");

  if (sdf->HasElement("direction") &&
      sdf->Get<std::string>("direction") == "reversed") {
    forward_force = -forward_force;
    reverse_force = -reverse_force;
  }

  std::cout  << "Initializing sim_piston: " << topic << " joint=" << joint->GetName()
        << " forward-force=" << forward_force
        << " reverse-force=" << reverse_force << std::endl;

  // Connect to Gazebo transport for messaging
 
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init("default");

  sub = node->Subscribe(topic, &PneumaticPiston::OnMsg, this);

  // Connect to the world update event.
  // This will trigger the Update function every Gazebo iteration
  updateConn = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&PneumaticPiston::Update, this, _1));
}

void PneumaticPiston::Update(const gazebo::common::UpdateInfo& info) {
  joint->SetForce(0, signal);
}

void PneumaticPiston::OnMsg(ConstVector3dPtr &msg) {
  double x=msg->x();
  if (x < -0.001) {
    signal = -reverse_force*fabs(x);
  } else if (x > 0.001) {
    signal = forward_force*fabs(x);
  }
}
