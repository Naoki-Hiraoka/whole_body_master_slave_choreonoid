#include <rtm/Manager.h>
#include "PrimitiveStateROSBridge.h"

void MyModuleInit(RTC::Manager* manager)
{
  PrimitiveStateROSBridgeInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent(("PrimitiveStateROSBridge?instance_name="+ros::this_node::getName().substr(1)).c_str()); // skip root name space for OpenRTM instance name

  return;
}

int main (int argc, char** argv)
{
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);
  ros::init(argc, argv, "PrimitiveStateROSBridge", ros::init_options::NoSigintHandler);

  // Initialize manager
  manager->init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  // manager->runManager(true);

  return 0;
}
