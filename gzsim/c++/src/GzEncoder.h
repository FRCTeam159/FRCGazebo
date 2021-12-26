#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzEncoder : public GzNode{
  static std::string topic_base;
  int chnl;
  double x;
  double y;
  void callback(const ConstVector3dPtr &msg);
  void setstate(std::string s);
  public:
    GzEncoder(int n,std::shared_ptr<nt::NetworkTable> t);
    ~GzEncoder();
    const std::string name() { return std::string("GzEncoder"+std::to_string(chnl));}
    bool connect();
    void reset();
    void run();
    void stop();
};