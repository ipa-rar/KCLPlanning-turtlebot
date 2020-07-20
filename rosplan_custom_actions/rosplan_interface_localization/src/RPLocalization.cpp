#include "rosplan_interface_localization/RPLocalization.h"

namespace KCL_rosplan
{

// constructor
RPLocalization::RPLocalization(ros::NodeHandle &nh, std::string turtlebot_name, std::string tf_prefix)
    : name(turtlebot_name), prefix(tf_prefix)
{

    // knowledge interface
    update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

    // costmap client
    clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    global_localization_client = nh.serviceClient<std_srvs::Empty>("/global_localization");

    // create publishers
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
}

// action dispatch callback
bool RPLocalization::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg)
{

    // localize the turtlebot
    // check action name
    if (0 == msg->name.compare("localise"))
    {
        ROS_INFO("KCL: (%s) action received", params.name.c_str());
        // Check robot name
        bool right_robot = false;
        for (size_t i = 0; i < msg->parameters.size(); i++)
        {
            if (0 == msg->parameters[i].key.compare("v") && 0 == msg->parameters[i].value.compare(name))
            {
                right_robot = true;
            }
        }
        if (!right_robot)
        {
            ROS_DEBUG("KCL: (Localiser) aborting action dispatch; handling robot %s", name.c_str());
            return false;
        }

        // reset AMCL particles
        std_srvs::Empty emptySrv;
        // global_localization_client.call(emptySrv);

        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.angular.x = base_cmd.angular.y = 0;
        base_cmd.angular.z = 0.5;

        double start = ros::WallTime::now().toSec();
        double time_diff = 0.0;
        double localization_time = 20.0;
        ros::Rate rate(10.0);
        while (ros::ok() && (time_diff < localization_time))
        {
            ros::spinOnce();
            cmd_vel_pub.publish(base_cmd);
            rate.sleep();
            time_diff = (ros::WallTime::now().toSec() - start);
            ROS_INFO("KCL: (Localiser) Localizing ... Time remaining : %.2f s ", (localization_time - time_diff));
        }

        base_cmd.angular.z = 0.0;
        cmd_vel_pub.publish(base_cmd);

        // get pose of the robot
        geometry_msgs::PoseStamped pBase, pMap;
        pBase.header.frame_id = tf::resolve(prefix, "base_link");
        pBase.pose.position.x = pBase.pose.position.y = pBase.pose.position.z = 0;
        pBase.pose.orientation.x = pBase.pose.orientation.y = pBase.pose.orientation.w = 0;
        pBase.pose.orientation.z = 1;

        try
        {
            tfl_.waitForTransform(tf::resolve(prefix, "base_link"), "map", ros::Time::now(), ros::Duration(1.0));
            tfl_.transformPose("map", pBase, pMap);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("KCL: (Localiser) transform error: %s", ex.what());
            return false;
        }

        double d = 10;
        std::string wpName = "";
        for (std::map<std::string, geometry_msgs::PoseStamped>::iterator wit = waypoints.begin(); wit != waypoints.end(); ++wit)
        {
            double vX = wit->second.pose.position.x - pMap.pose.position.x;
            double vY = wit->second.pose.position.y - pMap.pose.position.y;
            if (sqrt(vX * vX + vY * vY) < d)
            {
                wpName = wit->first;
                d = sqrt(vX * vX + vY * vY);
            }
        }
        std_msgs::String statement;
        if ("" == wpName)
        {
            ROS_WARN("KCL: (Localiser) no waypoint information found");

            statement.data = "Robot is lost ... ";
            return false;
        }
        else
        {
            ROS_INFO("KCL: (Localiser) continue");

            std::stringstream ss;
            ss << "Robot is close to " << wpName << std::endl;
            statement.data = ss.str();

            std_srvs::Empty emptySrv;
            clear_costmaps_client.call(emptySrv);

            // predicate localised
            rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
            updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
            updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            updatePredSrv.request.knowledge.attribute_name = "localised";
            diagnostic_msgs::KeyValue pair;
            pair.key = "v";
            pair.value = name;
            updatePredSrv.request.knowledge.values.push_back(pair);
            update_knowledge_client.call(updatePredSrv);

            // remove old robot_at
            updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
            updatePredSrv.request.knowledge.attribute_name = "robot_at";
            update_knowledge_client.call(updatePredSrv);

            // predicate robot_at
            updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
            updatePredSrv.request.knowledge.attribute_name = "robot_at";
            diagnostic_msgs::KeyValue pairWP;
            pairWP.key = "wp";
            pairWP.value = wpName;
            updatePredSrv.request.knowledge.values.push_back(pairWP);
            update_knowledge_client.call(updatePredSrv);
        }

        ROS_INFO("KCL: (Localiser) action complete");
        return true;
    }
}

/**
	 * parses a pose with yaw from strings: "[f, f, f]"
	 */
void RPLocalization::parsePose(geometry_msgs::PoseStamped &pose, std::string line)
{

    int curr, next;
    curr = line.find("[") + 1;
    next = line.find(",", curr);

    pose.pose.position.x = (double)atof(line.substr(curr, next - curr).c_str());
    curr = next + 1;
    next = line.find(",", curr);

    pose.pose.position.y = (double)atof(line.substr(curr, next - curr).c_str());
    curr = next + 1;
    next = line.find(",", curr);

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.w = (double)atof(line.substr(curr, next - curr).c_str());
    pose.pose.orientation.z = sqrt(1 - pose.pose.orientation.w * pose.pose.orientation.w);
}

bool RPLocalization::setupRoadmap(std::string filename)
{

    ros::NodeHandle nh("~");

    // clear previous roadmap from knowledge base
    ROS_INFO("KCL: (RPLocaliser) Loading roadmap from file %s", filename.c_str());

    // load configuration file
    std::ifstream infile(filename.c_str());
    std::string line;
    int curr, next;
    while (!infile.eof())
    {
        // read waypoint
        // std::cout << line << std::endl;
        std::getline(infile, line);
        curr = line.find("[");
        std::string name = line.substr(0, curr);

        // data
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        parsePose(pose, line);
        waypoints[name] = pose;
    }
    infile.close();
}
} // namespace KCL_rosplan

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rosplan_interface_localization");

    ros::NodeHandle nh("~");
    ros::NodeHandle nh2;

    //fetch ros params
    std::string filename;
    std::string turtlebot_name;
    std::string tf_prefix;
    nh.param("turtlebot_name", turtlebot_name, std::string("kenny"));
    nh.param("waypoint_file", filename, std::string("waypoints.txt"));

    nh2.getParam("/robot_state_publisher/tf_prefix", tf_prefix);

    // create PDDL action subscriber
    KCL_rosplan::RPLocalization rpl(nh, turtlebot_name, tf_prefix);
    rpl.setupRoadmap(filename);

    rpl.runActionInterface();

    return 0;
}
