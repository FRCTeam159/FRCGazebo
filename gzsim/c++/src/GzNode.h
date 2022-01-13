#pragma once

#pragma warning(push, 0)

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <ntcore.h>
#include <string>
#include <iostream>

enum {
  DISABLED=0,
  ENABLED = 1, 
  CONNECTED=2, 
  RUNNING=4, 
  RESET = 8
};
extern gazebo::transport::NodePtr gz_node;
class GzNode {
  protected:
    gazebo::transport::SubscriberPtr sub;
    gazebo::transport::PublisherPtr pub;
    std::shared_ptr<nt::NetworkTable> table;
    
    bool changed_state;
    int prev_state;
    void set_status_bit(int b);
    void clr_status_bit(int b);
  public:
    int status;
    std::string gz_topic;

    void getCtrlState();

    GzNode(std::string s,std::shared_ptr<nt::NetworkTable> t);
    virtual const std::string name();
    virtual bool connect(){ return false;}
    virtual void publish(){}
    virtual void subscribe(){}
    virtual void reset();
    virtual void stop();
    virtual void run();
    virtual void enable();
    virtual void disable();
    bool running() { return status & RUNNING;}
    bool stopped() { return running()?false:true;}
    bool connected() { return status & CONNECTED;}
    bool enabled() { return status & ENABLED;}
    bool disabled() { return enabled()?true:false;}
};