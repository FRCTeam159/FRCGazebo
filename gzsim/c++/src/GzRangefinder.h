#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzRangefinder : public GzNode{
  static std::string topic_base;
  void callback(const ConstVector3dPtr &msg);
  void setstate(std::string s);
  int chnl;
  public:
    GzRangefinder(int n,std::shared_ptr<nt::NetworkTable> t);
    ~GzRangefinder();
    const std::string name() { return std::string("GzGzRangefinder"+std::to_string(chnl));}
    bool connect();
    void reset();
    void run();
    void stop();
};