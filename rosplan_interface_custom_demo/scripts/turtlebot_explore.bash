#!/bin/bash

# call rosplan mapping interface to generate waypoints based on costmap
# echo "Generating waypoints.";
rosservice call /rosplan_roadmap_server/create_prm "{nr_waypoints: 2, min_distance: 0.3, casting_distance: 2.0, connecting_distance: 8.0, occupancy_threshold: 10, total_attempts: 10}";

# load waypoints into parameter server
# rosparam load `rospack find rosplan_interface_custom_demo`/config/waypoints.yaml;

# let RoadmapServer know that wps are available in param server
# rosservice call /rosplan_roadmap_server/load_waypoints;

# call turtlebot_explore_common.bash script
bash `rospack find rosplan_interface_custom_demo`/scripts/turtlebot_explore_common.bash


# rosservice call /rosplan_knowledge_base/update_array "
# $param_type
# $param"

# # NOTE: robot_at(kenny wp0) gets added by the mapping interface

# # automatically generate PDDL problem from KB snapshot (e.g. fetch knowledge from KB and create problem.pddl)

# echo "Calling problem generator.";
# rosservice call /rosplan_problem_interface/problem_generation_server;

# # make plan (e.g. call popf to create solution)
# echo "Calling planner interface.";
# rosservice call /rosplan_planner_interface/planning_server;

# # parse plan (parse console output and extract actions and params, e.g. create esterel graph)
# echo "Calling plan parser.";
# rosservice call /rosplan_parsing_interface/parse_plan;

# # dispatch (execute) plan. (send actions one by one to their respective interface and wait for them to finish)
# echo "Calling plan dispatcher.";
# rosservice call /rosplan_plan_dispatcher/dispatch_plan;

# echo "Finished!";