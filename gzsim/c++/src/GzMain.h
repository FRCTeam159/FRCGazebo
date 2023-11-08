#pragma once

#pragma warning(push, 0)

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_cpp.h>

#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <list>
#include <thread>

#include "GzNode.h"

class GzMain {
  std::shared_ptr<nt::NetworkTable> motors;
  std::shared_ptr<nt::NetworkTable> encoders;
  std::shared_ptr<nt::NetworkTable> cameras;
  std::shared_ptr<nt::NetworkTable> contacts;
  std::shared_ptr<nt::NetworkTable> gyros;
  std::shared_ptr<nt::NetworkTable> pistons;
  std::shared_ptr<nt::NetworkTable> rangefinders;
  std::shared_ptr<nt::NetworkTable> switches;

  std::shared_ptr<nt::NetworkTable> clock;
  std::list<GzNode *> nodes;

  nt::NetworkTableEntry cntrl_node;
  nt::NetworkTableEntry status_node;
  void sortKeys(std::vector<std::string> &keys);
  void genNodes();

 public:
  GzMain();
  ~GzMain();
  void init();
  void connect();
  void run();
  void setConnected(bool b);
};