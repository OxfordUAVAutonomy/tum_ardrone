// TODO: check these dependencies / includes

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/package.h>

#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include <sys/stat.h>
#include <string>

#include "AutopilotNode.h"
#include "AutopilotStructures.h"

const ControlCommand AutopilotNode::hoverCommand(0, 0, 0, 0);

AutopilotNode::AutopilotNode()
{
  // resolve channel names
  control_channel = nh_.resolveName("cmd_vel");
  dronepose_channel = nh_.resolveName("ardrone/predictedPose");
  command_channel = nh_.resolveName("tum_ardrone/com");
  takeoff_channel = nh_.resolveName("ardrone/takeoff");
  land_channel = nh_.resolveName("ardrone/land");

  // TODO: what's this exactly for?
  packagePath = ros::package::getPath("tum_ardrone");

  // TODO: make this more elegant
  // parse publishing frequency
  std::string val;
  float valFloat;
  ros::param::get("~minPublishFreq", val);
  if(val.size()>0)
    sscanf(val.c_str(), "%f", &valFloat);
  else
    valFloat = 110;
  minPublishFreq = valFloat;
  ROS_INFO("set minPublishFreq to %f ms", valFloat);

  // set up channels
  dronepose_sub = nh_.subscribe(dronepose_channel, 10, &AutopilotNode::droneposeCb, this);
  command_sub = nh_.subscribe(command_channel, 50, &AutopilotNode::commandCb, this);
  command_pub = nh_.advertise<std_msgs::String>(command_channel, 50);
  takeoff_pub = nh_.advertise<std_msgs::Empty>(takeoff_channel, 1);
  control_pub = nh_.advertise<geometry_msgs::Twist>(control_channel, 1);
  land_pub = nh_.advertise<std_msgs::Empty>(land_channel, 1);

  // initial state: autopilot is off
  isControlling = false;
}

AutopilotNode::~AutopilotNode()
{
  // TODO: check whether anything to be done here?
}

void AutopilotNode::commandCb(const std_msgs::StringConstPtr str)
{
  // only handle things beginning with c
  if(str->data.length() > 2 && str->data.substr(0,2) == "c ")
  {
    std::string cmd =str->data.substr(2,str->data.length()-2);

    // stop autopilot
    if(cmd.length() == 4 && cmd.substr(0,4) == "stop")
    {
      isControlling = false;
      // TODO: maybe generate AgentSpeak event?
      publishCommand("u l Autopilot: Stop Controlling");
      //ROS_INFO("STOP CONTROLLING!");
    }
    // start autopilot
    else if(cmd.length() == 5 && cmd.substr(0,5) == "start")
    {
      isControlling = true;
      // TODO: maybe generate AgentSpeak event?
      publishCommand("u l Autopilot: Start Controlling");
      //ROS_INFO("START CONTROLLING!");
    }
    else if(cmd.length() == 13 && cmd.substr(0,13) == "clearCommands")
    {
      // TODO: Implement!
      publishCommand("u l Autopilot: Clearing Commands");
      commandQueue.clear();
      clear();
    }
    else
    {
      commandQueue.push_back(cmd);
    }
  }
}

void AutopilotNode::droneposeCb(const tum_ardrone::filter_stateConstPtr statePtr)
{
  position.x = statePtr->x;
  position.y = statePtr->y;
  position.z = statePtr->z;
  position.yaw = statePtr->yaw;

  speed.dx = statePtr->dx;
  speed.dy = statePtr->dy;
  speed.dz = statePtr->dz;
  speed.dyaw = statePtr->dyaw;

  //scaleAccuracy = statePtr->scaleAccuracy;
}


void AutopilotNode::dynConfCb(tum_ardrone::AutopilotParamsConfig &config, uint32_t level)
{
  controller.Ki_gaz = config.Ki_gaz;
  controller.Kd_gaz = config.Kd_gaz;
  controller.Kp_gaz = config.Kp_gaz;

  controller.Ki_rp = config.Ki_rp;
  controller.Kd_rp = config.Kd_rp;
  controller.Kp_rp = config.Kp_rp;

  controller.Ki_yaw = config.Ki_yaw;
  controller.Kd_yaw = config.Kd_yaw;
  controller.Kp_yaw = config.Kp_yaw;

  controller.max_gaz_drop = config.max_gaz_drop;
  controller.max_gaz_rise = config.max_gaz_rise;
  controller.max_rp = config.max_rp;
  controller.max_yaw = config.max_yaw;
  controller.aggressiveness = config.aggressiveness;
  controller.rise_fac = config.rise_fac;
}

void AutopilotNode::parseFlightCommand(std::string command)
{
  ROS_INFO("executing command: %s",command.c_str());
  bool commandUnderstood = false;
  float parameters[4];

  // takeoff
  if(command == "takeoff")
  {
    // generate +!takeOff event in translated AgentSpeak
    achieve_takeOff();
    commandUnderstood = true;
  }
  // goto
  else if(sscanf(command.c_str(),"goto %f %f %f %f",
            &parameters[0], 
            &parameters[1], 
            &parameters[2], 
            &parameters[3]) == 4)
  {
    positiont target;
    target.x = parameters[0];
    target.y = parameters[1];
    target.z = parameters[2];
    target.yaw = parameters[3];
    // generate +!goto(Target) event in translated AgentSpeak
    achieve_goto(target);
    commandUnderstood = true;  
  }
  // goto variant
  else if(sscanf(command.c_str(),"moveByRel %f %f %f %f",
            &parameters[0], 
            &parameters[1], 
            &parameters[2],
            &parameters[3]) == 4)
  {
    positiont target;
    target.x = position.x + parameters[0];
    target.y = position.y + parameters[1];
    target.z = position.z + parameters[2];
    target.yaw = position.yaw + parameters[3];
    // generate +!goto(Target) event in translated AgentSpeak
    achieve_goto(target);
    commandUnderstood = true;
  }
  // land
  else if(command == "land")
  {
    // generate +!land event in translated AgentSpeak
    achieve_land();
    commandUnderstood = true;
  }
  // setInitialReachDist
  else if(sscanf(command.c_str(),"setInitialReachDist %f",
             &parameters[0]) == 1)
  {
    add_initialReachParameters(parameters[0], 5); // the original autopilot does only set xyzDist
    commandUnderstood = true;
  }
  // setStayWithinDist
  else if(sscanf(command.c_str(),"setStayWithinDist %f",
            &parameters[0]) == 1)
  {
    add_stayWithinParameters(parameters[0], 5); // the original autopilot does set xyzDist
    commandUnderstood = true;
  }
  // setStayTime
  else if(sscanf(command.c_str(),"setStayTime %f",
             &parameters[0]) == 1)
  {
    int stayTime = static_cast<int>(parameters[0]*1000); // convert to milliseconds
    add_stayTime(stayTime);
    commandUnderstood = true;
  }


  if(!commandUnderstood)
  {
    ROS_INFO("u l unknown command, skipping!");
  }
}

void AutopilotNode::mainLoop()
{
  // set up publishing frequency
  ros::Rate r(minPublishFreq);
  // for publishing some status updates at a lower frequency
  ros::Time lastStateUpdate = ros::Time::now();

  while (nh_.ok())
  {
    // callbacks
    ros::spinOnce();

    if (isControlling)
    {
      if (can_add_event() && commandQueue.size() > 0)
      {
        lastCommand = commandQueue.front();
        commandQueue.pop_front();
        parseFlightCommand(lastCommand);
      }
      // call next_step() of translated AgentSpeak
      next_step();
    }

   // send a status update around once in a while
   if((ros::Time::now() - lastStateUpdate) > ros::Duration(0.4))
   {
     reSendInfo();
     lastStateUpdate = ros::Time::now();
   }

    // wait
    r.sleep();
  }
}

pthread_mutex_t AutopilotNode::tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER;
void AutopilotNode::publishCommand(std::string c)
{
  std_msgs::String s;
  s.data = c.c_str();
  pthread_mutex_lock(&tum_ardrone_CS);
  command_pub.publish(s);
  pthread_mutex_unlock(&tum_ardrone_CS);
}

void AutopilotNode::reSendInfo()
{
  DronePosition t = controller.getLastTarget();
  TooN::Vector<4> e = controller.getLastError();
  ControlCommand c = controller.getLastSentControl();

  std::stringstream ss;
  ss << "u c ";
  ss << (isControlling ? "Controlling" : "Idle") << " (Queue: " << commandQueue.size() << ")" << "\n";
  ss << "Last: " << lastCommand << "\n";
  ss << "Next: " << ( commandQueue.size()>0 ? commandQueue.front() : "NONE") << "\n";
  ss << "Target: " << std::setprecision(2) << t.x << ", " << t.y << ", "<< t.z << ", " << t.yaw << "\n";
  ss << "Error: " << std::setprecision(2) << e[0] << ", " << e[1] << ", "<< e[2] << ", " << e[3] << "\n";
  ss << "Control: " << std::setprecision(2) << c.roll << ", " << c.pitch << ", " << c.gaz << ", " << c.yaw;
  publishCommand(ss.str());
}

DronePosition AutopilotNode::getPosition()
{
  return position;
}

DroneSpeed AutopilotNode::getSpeed()
{
  return speed;
}

ControlCommand AutopilotNode::calculateMovement(DronePosition position, DronePosition target)
{
  ControlCommand cmd = controller.getControlCommand(position, speed, target);/*, scaleAccuracy);*/
  return cmd;
}

void AutopilotNode::sendGoto(ControlCommand cmd)
{
  geometry_msgs::Twist cmdT;
  cmdT.angular.z = -cmd.yaw;
  cmdT.linear.z = cmd.gaz;
  cmdT.linear.x = -cmd.pitch;
  cmdT.linear.y = -cmd.roll;
  // TODO: in the original autopilot this was twice cmdT.angular.x, was this a bug?
  cmdT.angular.x = cmdT.angular.y = 0;

  control_pub.publish(cmdT);
}

void AutopilotNode::sendHover()
{
  sendGoto(hoverCommand);
}

void AutopilotNode::sendLand()
{
  land_pub.publish(std_msgs::Empty());
}

void AutopilotNode::sendTakeoff()
{
  takeoff_pub.publish(std_msgs::Empty());
}


