
#pragma warning(push, 0)

#include "sim_motor.h"

GZ_REGISTER_MODEL_PLUGIN(MotorPlugin)

void MotorPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  // Store the model pointer for convenience.
  this->model = model;

  if (model->GetJointCount() == 0) {
    std::cerr << "Invalid joint count, plugin not loaded\n";
    return;
  }

  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  std::string str = sdf->Get<std::string>("joint");
  //std::cerr << "Joint str:" << str << "\n";
  this->joint = model->GetJoint(str);
  if (this->joint == NULL) {
    std::cerr << "could not find Joint element in plugin - reverting to "
                 "first joint\n";
    this->joint =model->GetJoints()[0];
  } 
  // Setup a P-controller
  this->pid = gazebo::common::PID(0.1, 0, 0);

  // Apply the P-controller to the joint.
  this->model->GetJointController()->SetVelocityPID(
      this->joint->GetScopedName(), this->pid);

  this->SetVelocity(0.0);  // initial velocity

  // Create the node
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init(this->model->GetWorld()->Name());

  std::string topic;
  if (sdf->HasElement("topic"))
    topic = sdf->Get<std::string>("topic");
  else
    topic = "~/" + sdf->GetAttribute("name")->GetAsString();
  if (sdf->HasElement("multiplier"))
    multiplier = sdf->Get<double>("multiplier");
  else
    multiplier = 1;

  std::cout << "Initializing sim_motor: " << topic << " joint=" << joint->GetName()
        << " multiplier=" << multiplier << std::endl;
  // Subscribe to the topic, and register a callback
  this->sub = this->node->Subscribe(topic, &MotorPlugin::OnMsg, this);
}

void MotorPlugin::OnMsg(ConstVector3dPtr &_msg) { 
    this->SetVelocity(_msg->x()); 
}
/// \brief Set the velocity of the Velodyne
/// \param[in] _vel New target velocity

void MotorPlugin::SetVelocity(const double &_vel) {
  // std::cerr << "MotorPlugin::SetVelocity:" << _vel<<"\n";
  // Set the joint's target velocity.
  double vel = _vel;
  //vel = vel < -1 ? -1 : vel;
  //vel = vel > 1 ? 1 : vel;
  vel *= multiplier;
 
  this->model->GetJointController()->SetVelocityTarget(
      this->joint->GetScopedName(), vel);
}
