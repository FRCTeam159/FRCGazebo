#include "GzMain.h"

#include "GzMotor.h"
#include "GzEncoder.h"
#include "GzGyro.h"
#include "GzClock.h"
#include "GzCamera.h"

#include <condition_variable>
#include <mutex>

//using namespace nt;
using namespace std;

std::shared_ptr<nt::NetworkTable> table;
nt::NetworkTableInstance inst;
gazebo::transport::NodePtr gz_node(new gazebo::transport::Node());

GzMain::GzMain() {
  std::cout << "GzMain::GzMain()" << std::endl;
  table = inst.GetTable("gazebo");
  cntrl_node = table->GetEntry("GzCntrl");
  status_node = table->GetEntry("GzStatus");
}
GzMain::~GzMain() {
  std::cout << "GzMain::~GzMain()" << std::endl;
  nodes.clear();
}
// ========================================================
// void GzMain::init() wait for Robot program to generate sim objects
// ========================================================
// Uses a network string object called "GzCntrl" in gazebo subtable
// initialized to the value "init"
// - Assumes all objects are generated in RobotInit via appropriate
//   constructors (subsystems etc)
// - GzMain::init waits until the value of GzCntrl is changed
// - at the end of RobotInit (or the beginning of SimulationInit)
//   Robot program changes the value of GzCntrl to "run"
// - GzMain::init then calls GzMain::genNodes() to genetate the 
//   c++ node objects
// ========================================================
void GzMain::init() {
  // wait for java to initialize all NT modes
  while (true) {
    std::string s = cntrl_node.GetString("init");
    if (s == "run") 
      break;
    std::this_thread::sleep_for(chrono::milliseconds(20));
  }
  // generate nodes
  genNodes();
  gz_node->Init();
}

// ========================================================
// void GzMain::genNodes() generate GzNode objects
// ========================================================
// Load a network SubTable called "gazebo"
// for each node type {GzMotor,GzEncoder,GzGyro ..}
//   look for a "type" SubTable in gazebo of the form:
//     "motor", "encoder", ...
//   for each type SubTable (encoder, motor)
//   look for a "channel" SubTable in gazebo of the form:
//      "1","2" ..
//   each "channel" subtable may contain the following event types
//   - double "pub": data from the Robot program to send to Gazebo (up to 3 values)
//   - string "cntrl": a command string (e.g. "new, "disable","reset")
//   - sub: a Gazebo subscriber item eg. data to send to the Robot program from Gazebo
//   Generate a new GzNode of the approriate type (GzMotor etc.) 
//   - add the GzNode pointer to a list of GzNodes
// ========================================================
void GzMain::genNodes() {
  // build motors
  motors = table->GetSubTable("motor");
  std::vector<std::string> keys = motors->GetSubTables();
    sortKeys(keys);
  std::cout << "motors:" << keys.size() << std::endl;
  for (int i = 0; i < keys.size(); i++) {
    std::shared_ptr<nt::NetworkTable> tbl = motors->GetSubTable(keys[i]);
    int j = std::stoi(keys[i]);
    nodes.emplace_back(new GzMotor(j, tbl));
  }
  // build encoders
  keys.clear();
  encoders = table->GetSubTable("encoder");
  keys = encoders->GetSubTables();
  sortKeys(keys);
  std::cout << "encoders:" << keys.size() << std::endl;
  for (int i = 0; i < keys.size(); i++) {
    std::shared_ptr<nt::NetworkTable> tbl = encoders->GetSubTable(keys[i]);
    int j = std::stoi(keys[i]);
    nodes.emplace_back(new GzEncoder(j, tbl));
  }

  // build cameras
  keys.clear();
  cameras = table->GetSubTable("camera");
  keys = cameras->GetSubTables();
  sortKeys(keys);
  std::cout << "cameras:" << keys.size() << std::endl;
  for (int i = 0; i < keys.size(); i++) {
    std::shared_ptr<nt::NetworkTable> tbl = cameras->GetSubTable(keys[i]);
    int j = std::stoi(keys[i]);
    nodes.emplace_back(new GzCamera(j, tbl));
  }
  // build gyro
  gyro=table->GetSubTable("gyro");
  nodes.emplace_back(new GzGyro(gyro));

   // build clock
  clock=table->GetSubTable("clock");
  nodes.emplace_back(new GzClock(clock));
}
// ========================================================
void GzMain::sortKeys(std::vector<std::string> &keys){
// for some unknown reason "motors","encoders" subtables contains duplicates of each
// channel assigned from Java. work-around is to sort the channel names and
// throw away the duplicates
  sort(keys.begin(), keys.end());
  keys.erase(unique(keys.begin(), keys.end()), keys.end());
}
// ========================================================
// void GzMain::connect() connect GzNode objects to Gazebo
// ========================================================
void GzMain::connect() {
  for (GzNode *node : nodes) {
    node->connect();
  }
}
// ========================================================
// void GzMain::run() run until the simulation exits
// ========================================================
// relay data messages from the Robot program to Gazebo (publish)
// relay data from Gazebo to the Robot program (subscribe)
// ========================================================
void GzMain::run() {
  while (true) {
    // call publishers and subscribers
    for (GzNode *node : nodes) {
      node->getCtrlState();
      if (node->status & CONNECTED) 
        node->publish();
    }
    this_thread::sleep_for(chrono::milliseconds(20));
  }
}

// ========================================================
// void GzMain::setConnected() set Gazebo connection status
// ========================================================
// - send state to the robot program via a network 
//   string node called "GzStatus"
// ========================================================
void GzMain::setConnected(bool b) {
  if (b)
    status_node.SetString("connected");
  else
    status_node.SetString("not-connected");
  inst.Flush();
}
// ========================================================
// try to connect to Gazebo with a 1 sec timeout
// - this code prevents the main thread from hanging if 
//   Gazebo isn't running
// ========================================================
int try_to_connect_gazebo() {
  std::mutex m;
  std::condition_variable cv;
  bool retValue = false;

  std::thread t([&cv, &retValue]() {
    retValue = gazebo::client::setup();
    cv.notify_one();
  });
  t.detach();
  {
    std::unique_lock<std::mutex> l(m);
    if (cv.wait_for(l, 1s) == std::cv_status::timeout)
      throw std::runtime_error("Timeout");
  }
  return retValue;
}
// ========================================================
// main entry point for program start
// ========================================================
int main(int argc, char *argv[]) {
  int connected = false;

  std::cout << "attempting to connect to Gazebo" << std::endl;
  connected = try_to_connect_gazebo();

  std::cout << "Gazebo NTClient started" << std::endl;
  inst = nt::NetworkTableInstance::GetDefault();
  inst.SetUpdateRate(0.02);
  inst.StartClient("localhost");
  GzMain *gzmain = new GzMain();
  // inform robot program if Gazebo is running or not
  gzmain->setConnected(connected);
  if (connected) {
    std::cout << "Gazebo connected" << std::endl;
    gzmain->init();
    gzmain->connect();
    gzmain->run(); // run until program exits
   
  } else {
    std::cout << "failed to connect to Gazebo - Is Gazebo running ?"
              << std::endl;
  }
  delete gzmain;
  if (connected) 
    gazebo::client::shutdown();
}