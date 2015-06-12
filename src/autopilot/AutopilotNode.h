#pragma once

#ifndef __AUTOPILOT_NODE_H
#define __AUTOPILOT_NODE_H

// TODO: check these dependencies / includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include "tum_ardrone/AutopilotParamsConfig.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

#include "AutopilotStructures.h"
#include "PIDController.h"

extern "C" {
  #include "autopilotAI/autopilot.h"
}

class AutopilotNode
{
private:
  ros::Subscriber dronepose_sub;
  ros::Publisher control_pub;
  ros::Subscriber command_sub;
  ros::Publisher command_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher toggleState_pub;

  ros::NodeHandle nh_;
  static pthread_mutex_t tum_ardrone_CS;

  DronePosition position;
  DroneSpeed speed;
  //double scaleAccuracy;

  // parameters
  bool isControlling;
  int minPublishFreq;
  std::string packagePath;

  std::string dronepose_channel;
  std::string command_channel;

  std::string land_channel;
  std::string control_channel;
  std::string takeoff_channel;

  PIDController controller;

  std::deque<std::string> commandQueue;
  std::string lastCommand;

  void parseFlightCommand(std::string command);

public:
  AutopilotNode();
  ~AutopilotNode();

  static const ControlCommand hoverCommand;

  // ROS message callbacks
  void commandCb(const std_msgs::StringConstPtr str);
  void dynConfCb(tum_ardrone::AutopilotParamsConfig &config, uint32_t level);
  void droneposeCb(const tum_ardrone::filter_stateConstPtr statePtr);
  
  // main pose-estimation loop
  void mainLoop();

  // writes a string message to "/tum_ardrone/com".
  // is thread-safe (can be called by any thread, but may block till other calling thread finishes)
  void publishCommand(std::string c);
  // publish info about autopilot state (displayed by GUI)
  void reSendInfo();

  DronePosition getPosition();
  DroneSpeed getSpeed();

  ControlCommand calculateMovement(DronePosition position, DronePosition target);

  // control drone functions
  void sendGoto(ControlCommand cmd);
  void sendHover();
  void sendLand();
  void sendTakeoff();
};
#endif /* __AUTOPILOT_NODE_H */
