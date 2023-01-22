#pragma warning(push, 0)

#include "sim_contact.h"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr sdf)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }
  contact_made=false;

  if (sdf->HasElement("topic")) {
    topic = sdf->Get<std::string>("topic");
  } else {
    topic = "~/" + sdf->GetAttribute("name")->GetAsString();
  }

  std::cout << "Initializing sim_contact: " << topic << std::endl;

  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init("default");
  pub = node->Advertise<gazebo::msgs::Vector3d>(topic);

  // Connect to the sensor update event.
  updateConn = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::Update, this));

  // Make sure the parent sensor is active.
  parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::Update()
{
  // Get all the contacts.

gazebo::msgs::Vector3d msg;
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  bool have_contact=contacts.contact_size()>0;
  bool new_state=false;

  if(have_contact){
    if(!contact_made){
        std::cout << "contact made"<<std::endl;
        new_state=true;
        contact_made=true;
    }
    contact_made=true;
  }else{
      if(contact_made){
          std::cout << "contact lost"<<std::endl;
          new_state=true;
          contact_made=false;
      }
      contact_made=false;
  }
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(have_contact, new_state, contact_made));
  pub->Publish(msg);
}