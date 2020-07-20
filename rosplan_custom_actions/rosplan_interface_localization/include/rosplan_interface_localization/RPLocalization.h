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

#ifndef KCL_localise
#define KCL_localise

/**
 * This file defines the RPLocaliser class.
 * RPLocaliser is used to slowly rotate the base for localisation.
 * PDDL "localise" action is listened for.
 */

namespace KCL_rosplan
{

class RPLocalization : public RPActionInterface
{

public:
  /**
  * @brief constructor
  */
  RPLocalization(ros::NodeHandle &nh, std::string turtlebot_name, std::string tf_prefix);
  bool setupRoadmap(std::string filename);
  /**
         * @brief listen to and process action_dispatch topic
         * @param msg this parameter comes from the topic subscription, it contains the request (id, name, params, etc)
         * @return true if execution was successful
         */
  bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);

private:
  tf::TransformListener tfl_;
  ros::ServiceClient update_knowledge_client;
  ros::ServiceClient clear_costmaps_client;
  ros::ServiceClient global_localization_client;
  ros::Publisher action_feedback_pub;
  ros::Publisher cmd_vel_pub;

  std::map<std::string, geometry_msgs::PoseStamped> waypoints;
  std::string name;
  std::string prefix;

  void parsePose(geometry_msgs::PoseStamped &pose, std::string line);
};
} // namespace KCL_rosplan
#endif
