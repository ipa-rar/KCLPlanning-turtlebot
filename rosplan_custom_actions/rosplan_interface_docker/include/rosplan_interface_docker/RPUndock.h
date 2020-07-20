#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <rosplan_action_interface/RPActionInterface.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#ifndef KCL_UNDOCK
#define KCL_UNDOCK

/**
 * This file defines the RPLocaliser class.
 * RPLocaliser is used to slowly rotate the base for localisation.
 * PDDL "localise" action is listened for.
 */

namespace KCL_rosplan
{

class RPUndock : public RPActionInterface
{

public:
  /**
  * @brief constructor
  */
  RPUndock(ros::NodeHandle &nh, std::string turtlebot_name, std::string tf_prefix);
  /**
         * @brief listen to and process action_dispatch topic
         * @param msg this parameter comes from the topic subscription, it contains the request (id, name, params, etc)
         * @return true if execution was successful
         */
  bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);

private:
  tf::TransformListener tf_listener;

  ros::ServiceClient update_knowledge_client;
  ros::Publisher cmd_vel_pub;

  std::string name;
  std::string prefix;
};
} // namespace KCL_rosplan
#endif
