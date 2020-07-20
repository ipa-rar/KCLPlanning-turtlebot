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
#include <ar_dock_motion_server/DockMotionAction.h>

#ifndef KCL_DOCK
#define KCL_DOCK

/**
 * This file defines the RPLocaliser class.
 * RPLocaliser is used to slowly rotate the base for localisation.
 * PDDL "localise" action is listened for.
 */

namespace KCL_rosplan
{

class RPDock : public RPActionInterface
{

public:
  /**
  * @brief constructor
  */
  RPDock(ros::NodeHandle &nh, std::string turtlebot_name);
  /**
         * @brief listen to and process action_dispatch topic
         * @param msg this parameter comes from the topic subscription, it contains the request (id, name, params, etc)
         * @return true if execution was successful
         */
  bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);

private:
  ros::ServiceClient update_knowledge_client;
  ros::Publisher cmd_vel_pub;
  actionlib::SimpleActionClient<ar_dock_motion_server::DockMotionAction> action_client;

  std::string name;
};
} // namespace KCL_rosplan
#endif
