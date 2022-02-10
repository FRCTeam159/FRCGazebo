
#pragma warning(push, 0)

#include "sim_motor.h"

#define ROTATIONS 0.5/M_PI;

GZ_REGISTER_MODEL_PLUGIN(MotorPlugin)

void MotorPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Store the model pointer for convenience.
  this->model = model;
  debug = false;

  if (model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, plugin not loaded\n";
    return;
  }

  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  std::string str = sdf->Get<std::string>("joint");
  // std::cerr << "Joint str:" << str << "\n";
  this->joint = model->GetJoint(str);
  if (this->joint == NULL)
  {
    std::cerr << "could not find Joint element in plugin - reverting to "
                 "first joint\n";
    this->joint = model->GetJoints()[0];
  }

  // Create the node
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init("default");

  std::string topic;
  if (sdf->HasElement("topic"))
    topic = sdf->Get<std::string>("topic");
  else
    topic = "~/" + sdf->GetAttribute("name")->GetAsString();
  if (sdf->HasElement("multiplier"))
    multiplier = sdf->Get<double>("multiplier");
  else
    multiplier = 1;
  // Setup a P-controller

  double P = 0.1;
  double I = 0.0;
  double D = 0.0;
  if (sdf->HasElement("P"))
    P = sdf->Get<double>("P");
  if (sdf->HasElement("I")){
    I = sdf->Get<double>("I");
    debug = true;
  }
  if (sdf->HasElement("D"))
    D = sdf->Get<double>("D");
  this->pid = gazebo::common::PID(P, I, D,100000,-100000);

  // Apply the P-controller to the joint.
  this->model->GetJointController()->SetVelocityPID(
      this->joint->GetScopedName(), this->pid);

  last_vel = 1;
  scale = 1;
  this->SetVelocity(0); // initial velocity

  std::cout << "Initializing sim_motor: " << topic << " joint=" << joint->GetName()
            << " multiplier=" << multiplier << " I=" << I << std::endl;
  // Subscribe to the topic, and register a callback
  this->sub = this->node->Subscribe(topic, &MotorPlugin::OnMsg, this);
}

void MotorPlugin::OnMsg(ConstVector3dPtr &_msg)
{
  scale=_msg->y();
  double rps=_msg->x()*2*M_PI;
  double vel = rps*multiplier * scale;

  if(debug && vel !=last_vel){
    int is_neg=signbit(vel);
    int was_neg=signbit(last_vel);
    if(is_neg != was_neg){
      std::cout << "PID.Reset vel:"<< vel<<" last_vel:"<<last_vel<<std::endl;
      pid.Reset();
    }
  }
  last_vel = vel;
  this->SetVelocity(vel);
}
/// \brief Set the velocity of the Velodyne
/// \param[in] _vel New target velocity

//#define USE_THE_FORCE
void MotorPlugin::SetVelocity(double vel)
{
#ifdef USE_THE_FORCE
    this->joint->SetForce(0, vel);
#else
    //if (debug){
      //double jvel=joint->GetVelocity(0);
      //std::cout << "set vel:" << vel << " actual vel:" << jvel << " err:" << jvel-vel << std::endl;
    //} 
  this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), vel);
  
#endif
}
