#include <neuro_car/sigint_handler.h>

volatile sig_atomic_t exitFlag = 0;

void exitInterrupt([[maybe_unused]] int sig) { exitFlag = 1; }

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params,
                      XmlRpc::XmlRpcValue& result) {
  // https://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    num_params = params.size();
  }
  if (num_params > 1) {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    exitFlag = 1;  // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

void handleInterrupts(int argc, char** argv, const char* name, bool anonymous) {
  // Register signal handler
  signal(SIGINT, exitInterrupt);

  if (anonymous) {
    ros::init(
        argc, argv, name,
        ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  } else {
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
  }

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
}
