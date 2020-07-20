#include "rosplan_interface_docker/RPDock.h"

namespace KCL_rosplan
{

// constructor
RPDock::RPDock(ros::NodeHandle &nh, std::string turtlebot_name)
    : name(turtlebot_name), action_client("/ar_dock_motion_server", true)
{
    ROS_INFO("KCL: (Docker) waiting for action server to start on /ar_dock_motion_server");
    action_client.waitForServer();

    // knowledge interface
    update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
    // create publishers
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
}

// action dispatch callback
bool RPDock::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg)
{

    // dock the turtlebot
    // check action name
    if (0 == msg->name.compare("dock"))
    {
        ROS_INFO("KCL: (%s) action received", params.name.c_str());

        // Check robot name
        bool right_robot = false;
        for (size_t i = 0; i < msg->parameters.size(); i++)
        {
            if (0 == msg->parameters[i].value.compare(name))
            {
                right_robot = true;
            }
        }
        if (!right_robot)
        {
            ROS_WARN("KCL: (Dock) aborting action dispatch; handling robot %s", name.c_str());
            return false;
        }

        ar_dock_motion_server::DockMotionGoal goal;
        goal.start = true;
        action_client.sendGoal(goal);
        ROS_INFO("KCL: (Dock) action client:Sending dock action command");

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(60.0));
        ROS_INFO("KCL: (Dock) action client: Wait for result ... ");

        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0.0;
        base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0.0;

        if (finished_before_timeout)
        {
            cmd_vel_pub.publish(base_cmd);
            ROS_INFO("KCL: (Dock) action complete - dock");
            return true;
        }
        else
        {
            cmd_vel_pub.publish(base_cmd);
            ROS_INFO("KCL: (Dock) action FAILED - dock");
            return true;
        }
    }
}

} // namespace KCL_rosplan

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rosplan_interface_dock");

    ros::NodeHandle nh("~");

    //fetch ros params
    std::string turtlebot_name;

    nh.param("turtlebot_name", turtlebot_name, std::string("kenny"));

    // create PDDL action subscriber
    KCL_rosplan::RPDock rpdk(nh, turtlebot_name);

    rpdk.runActionInterface();

    return 0;
}
