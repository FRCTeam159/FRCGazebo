
#pragma warning(push, 0)

#define M_PI 3.14159265358979323846

#include <gazebo/gazebo_config.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore.h>

#include <chrono>
#include <cstdlib>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/transport.hh>
#include <iostream>
#include <thread>

using namespace std;
using namespace nt;
using std::shared_ptr;

std::shared_ptr<nt::NetworkTable> table=NULL;

static void encoder1_msg(const ConstVector3dPtr &msg){
  if(table !=NULL){
    table->PutNumber("motor1-position",msg->x());
    table->PutNumber("motor1-velocity",msg->y());
  }
}
static void encoder2_msg(const ConstVector3dPtr &msg){
  if(table !=NULL){
    table->PutNumber("motor2-position",msg->x());
    table->PutNumber("motor2-velocity",msg->y());
  }
}
static void gyro_msg(const ConstVector3dPtr &msg){
  if(table !=NULL){
    table->PutNumber("gyro-angle",msg->x());
    table->PutNumber("gyro-velocity",msg->y());
  }
}
/////////////////////////////////////////////////
int main(int _argc, char **_argv) {
  double val = std::atof(_argv[1]);

  std::string topic1 = "~/gazebo/frc/simulator/motor/1";
  std::string topic2 = "~/gazebo/frc/simulator/motor/2";
  std::string topic3 = "~/gazebo/frc/simulator/encoder/1";
  std::string topic4 = "~/gazebo/frc/simulator/encoder/2";
  std::string topic5 = "~/gazebo/frc/simulator/gyro";


  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::PublisherPtr motor1_pub =
      node->Advertise<gazebo::msgs::Vector3d>(topic1);

  // Wait for a subscriber to connect to this publisher
  bool have_motor1 = motor1_pub->WaitForConnection(gazebo::common::Time(2, 0));

  if (!have_motor1)
    std::cerr << "connection failed for motor1\n";
  else
    std::cerr << "connection suceeded for motor1\n";
  gazebo::transport::PublisherPtr motor2_pub =
      node->Advertise<gazebo::msgs::Vector3d>(topic2);
  bool have_motor2 = motor2_pub->WaitForConnection(gazebo::common::Time(2, 0));

  if (!have_motor2)
    std::cerr << "connection failed for motor2\n";
  else
    std::cerr << "connection suceeded for motor2\n";
  gazebo::msgs::Vector3d msg;
 
  auto inst = NetworkTableInstance::GetDefault();
  table = inst.GetTable("gazebo");

  cout << "CNTClientTest started" << endl;
  inst.StartClient("localhost");

  gazebo::transport::SubscriberPtr encoder1_sub=node->Subscribe(topic3,encoder1_msg);
  gazebo::transport::SubscriberPtr encoder2_sub=node->Subscribe(topic4,encoder2_msg);
  gazebo::transport::SubscriberPtr gyro_sub=node->Subscribe(topic5,gyro_msg);

  while (true) {
    double x = 0;
    if(have_motor1){
      x = table->GetNumber("motor1", 0.0);
      gazebo::msgs::Set(&msg, ignition::math::Vector3d(x, 0, 0));
      motor1_pub->Publish(msg);
    }  
    if(have_motor2){
      x = table->GetNumber("motor2", 0.0);
      gazebo::msgs::Set(&msg, ignition::math::Vector3d(x, 0, 0));
      motor2_pub->Publish(msg);
    }  
    this_thread::sleep_for(chrono::milliseconds(20));
  }
  gazebo::client::shutdown();
}
