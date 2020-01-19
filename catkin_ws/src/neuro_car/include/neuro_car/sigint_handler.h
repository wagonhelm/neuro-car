#pragma once

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>

// Signal processing
extern volatile sig_atomic_t exitFlag;

// Declarations
void exitInterrupt(int sig);
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

// Call this function instead of ros::init in order to automatically handle
// SIGINT  When SIGINT is received, exitFlag will be set
void handleInterrupts(int argc, char** argv, const char* name,
                      bool anonymous = false);
