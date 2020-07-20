## Package summary

- 3rd_party 
    - [ROSPlan related packages]
        - [Occupancy_grid_utils](https://github.com/clearpathrobotics/occupancy_grid_utils)
        - [ROSPlan](https://github.com/KCL-Planning/ROSPlan)
        - [rosplan_demos](https://github.com/KCL-Planning/rosplan_demos)
    - [Turtlebot3 related packages]
        - [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
        - [turtlebot3_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications)
        - [turtlebot3_applications_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs)
        - [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
        - [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
    - [AR marker Visual Detection]
        - [ar_track_alvar](https://github.com/ros-perception/ar_track_alvar)

- rosplan_custom_actions
    - [Custom action interfaces related packages]
        - rosplan_interface_localization
        - rosplan_interface_docker
        - ar_dock_motion_server

- rosplan_interface_custom_demo
- ar_marker_17

---
## Dependencies 
Please check and install the dependecies:

For [ROSPlan](https://github.com/KCL-Planning/ROSPlan):
> sudo apt install flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-$ROS_DISTRO-tf2-bullet

For [Turtlebot3 ROS packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-1-packages):
> please change distro from kinetic to melodic 

**NOTE:** Before executing any turtlebot command, please add `ONE` of the following line to your ~/.bashrc
```
export TURTLEBOT3_MODEL=burger
export TURTLEBOT3_MODEL=waffle
export TURTLEBOT3_MODEL=waffle_pi
```

**NOTE:** Please copy the `ar_marker_17` folder to `~/.gazebo/models`  
This marker models is the 'docking station' spot for turtlebot3 in Gazebo

---

## Teleop control 
### This is to check your turtlebot3 and gazebo installation installation    
**[In gazebo]** You should see turtlebot3 is moving according to your keyboard input  
**[On rviz]** You should be able to observe map, cost map and lazer scan points on the GUI  

`Terminal 1 -- To bringup Gazebo simulator and Rviz`
```
roslaunch rosplan_interface_custom_demo simulated_actions.launch 
```
`Terminal 2 -- Use 'w' , 'a' , 's' , 'd' to move your robot `
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

--- 

## ROSplan + test [undock, localize , dock] custom action interface 
This turtblebot will execute rosplan actions [undock, localize, dock ] in the ROSplan in Gazebo  

`Terminal 1 -- To bringup Gazebo simulator and Rviz with ROSplan custom action interfaces`
```
roslaunch rosplan_interface_custom_demo turtlebot3_dock_loc_undock.launch
```

`Terminal 2 -- Generate plans`
```
cd {YOUR_WORKSPACE_NAME}/src/rosplan_interface_custom_demo/scripts/
./simulated_actions.bash
```

--- 

## ROSplan + test [goto_waypoints] action interface 
This turtblebot will execute rosplan actions [goto_waypoints] in the ROSplan in Gazebo  

`Terminal 1 -- To bringup Gazebo simulator and Rviz with ROSplan custom action interfaces`
```
roslaunch rosplan_interface_custom_demo simulated_actions.launch 
```

`Terminal 2 -- Generate plans`
```
cd {YOUR_WORKSPACE_NAME}/src/rosplan_interface_custom_demo/scripts/
./simulated_actions.bash
```
---

## Current Issues

1. auto plan generation issue:
    - valid plan can be generated only when using the 4 cmd in `simulated_action.bash`
    - when insert waypoint the plan is unsolvable
    - problem raised at this [link](https://github.com/KCL-Planning/ROSPlan/issues/257)

2. Things to optimize:
    1) Dock action is not perfect due to using visual marker
	    - once the robot is near the dock , marker will disappear from camera which causes marker location undefine
        - need to develop extra module to detect the robot is already at docking station

    2) Launching by bash file 
	    - need to convert into python script/ launch file with service node
