#include "rosplan_interface_docker/RPUndock.h"

namespace KCL_rosplan
{

// constructor
RPUndock::RPUndock(ros::NodeHandle &nh, std::string turtlebot_name, std::string tf_prefix)
    : name(turtlebot_name), prefix(tf_prefix)
{

    // knowledge interface
    update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
    // create publishers
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
}

// action dispatch callback
bool RPUndock::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg)
{

    // localize the turtlebot
    // check action name
    if (0 == msg->name.compare("undock"))
    {
        ROS_INFO("KCL: (%s) action received", params.name.c_str());

        // rosplan_dispatch_msgs::ActionFeedback fb;
        // fb.action_id = msg->action_id;
        // fb.status = "action enabled";
        // action_feedback_pub.publish(fb);

        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0.0;
        base_cmd.angular.x = base_cmd.angular.y = base_cmd.angular.z = 0.0;

        base_cmd.linear.x = -0.1;
        tf::StampedTransform init_transform, curr_transform;

        try
        {
            tf_listener.waitForTransform(tf::resolve(prefix, "base_link"), "map", ros::Time::now(), ros::Duration(1.0));

            tf_listener.lookupTransform(tf::resolve(prefix, "base_link"), "map",
                                        ros::Time(0), init_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }

        double start = ros::WallTime::now().toSec();
        ros::Rate rate(10.0);
        float distance = 0.0;
        float target_distance = 0.3; // move backward 0.3m from the docking point

        // while (ros::ok() && (ros::WallTime::now().toSec() - start < 30.0))
        while (ros::ok() && (distance < target_distance))

        {
            ros::spinOnce();
            cmd_vel_pub.publish(base_cmd);
            rate.sleep();
            tf_listener.lookupTransform(tf::resolve(prefix, "base_link"), "map",
                                        ros::Time(0), curr_transform);
            float curr_x = curr_transform.getOrigin().x();
            float curr_y = curr_transform.getOrigin().y();
            float diff_x = init_transform.getOrigin().x() - curr_transform.getOrigin().x();
            float diff_y = init_transform.getOrigin().y() - curr_transform.getOrigin().y();

            distance = sqrt(diff_x * diff_x + diff_y * diff_y);
            ROS_INFO("KCL: (Undock) Undocking ... moved %.2f m away from docking point ", distance);
        }

        //end the reverse cmd_vel control
        base_cmd.linear.x = 0.0;
        cmd_vel_pub.publish(base_cmd);

        ROS_INFO("KCL: (Undock) action complete - undock");

        // add predicate
        rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
        updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
        updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        updatePredSrv.request.knowledge.attribute_name = "undocked";
        diagnostic_msgs::KeyValue pair;
        pair.key = "v";
        pair.value = name;
        updatePredSrv.request.knowledge.values.push_back(pair);
        update_knowledge_client.call(updatePredSrv);

        // remove predicate
        updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
        updatePredSrv.request.knowledge.attribute_name = "docked";
        update_knowledge_client.call(updatePredSrv);

        // ros::spinOnce();
        // ros::Rate big_rate(0.5);
        // big_rate.sleep();

        // // publish feedback (achieved)
        // fb.status = "action achieved";
        // action_feedback_pub.publish(fb);
        // return true;
    }
}

} // namespace KCL_rosplan

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rosplan_interface_undock");

    ros::NodeHandle nh("~");
    ros::NodeHandle nh2;

    //fetch ros params
    std::string turtlebot_name;
    std::string tf_prefix;

    nh.param("turtlebot_name", turtlebot_name, std::string("kenny"));
    nh2.getParam("/robot_state_publisher/tf_prefix", tf_prefix);

    // create PDDL action subscriber
    KCL_rosplan::RPUndock rpudk(nh, turtlebot_name, tf_prefix);

    rpudk.runActionInterface();

    return 0;
}
