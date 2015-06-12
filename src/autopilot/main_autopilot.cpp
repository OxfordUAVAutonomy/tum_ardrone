#include <string>
#include <sstream>

#include "ros/ros.h"
#include "ros/package.h"
#include "../HelperFunctions.h"

#include "AutopilotNode.h"
#include "AutopilotStructures.h"

extern "C" {
  #include "autopilotAI/actions.h"
  #include "autopilotAI/types.h"
}

// this global var is used in getMS(ros::Time t) to convert to a consistent integer timestamp used internally pretty much everywhere.
// kind of an artifact from Windows-Version, where only that was available / used.
unsigned int ros_header_timestamp_base = 0;

// TODO: this is an ugly solution, improve!
AutopilotNode *node;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autopilot");

  ROS_INFO("Starting Autopilot Node.");

  AutopilotNode autopilotNode;

  node = &autopilotNode;

  dynamic_reconfigure::Server<tum_ardrone::AutopilotParamsConfig> srv;
  dynamic_reconfigure::Server<tum_ardrone::AutopilotParamsConfig>::CallbackType f;
  f = boost::bind(&AutopilotNode::dynConfCb, &autopilotNode, _1, _2);
  srv.setCallback(f);

  // call init() of translated AgentSpeak
  init();

  autopilotNode.mainLoop();

  return 0;
}

void sendGoto(control_commandt cmd)
{
  node->sendGoto(cmd);
}

void sendHover()
{
  node->sendHover();
}

void sendLand()
{
  node->sendLand();
}

void sendTakeOff()
{
  node->sendTakeoff();
}

control_commandt calculateMovement(positiont position, positiont target)
{
  ControlCommand command = node->calculateMovement(position, target);
  control_commandt cmd;
  cmd.roll = command.roll;
  cmd.pitch = command.pitch;
  cmd.yaw = command.yaw; 
  cmd.gaz = command.gaz;
  return cmd;
}

bool closeEnough(positiont position, positiont target, double xyzDist, double yawDist)
{
  positiont diff = getDifference(position, target);
  return ((getNormSquared(diff) < xyzDist * xyzDist) && (diff.yaw * diff.yaw < yawDist * yawDist));
}

int getCurrentTimeMS()
{
  return getMS();
}

bool longEnough(int startTime, int waitTime)
{
  int time_diff = getMS() - startTime;
  return (time_diff >= waitTime);
}

void notifyUser(messaget msg)
{
  // log messages are displayed by the UI node
  std::stringstream message;
  message << "u l notification: " << msg;
  node->publishCommand(message.str());
  // alternatively, you can use
  //ROS_INFO("notification: %s", msg);
}

void updateBeliefs(void) {
  DronePosition position = node->getPosition();
  positiont pos;
  pos.x = position.x;
  pos.y = position.y;
  pos.z = position.z;
  pos.yaw = position.yaw;
  set_myPosition(pos);
}

