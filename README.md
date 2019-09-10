# Lazy Theta* 2D 

In this repository you can find the library of the Lazy Theta* with Optimizations and Cost Adapted of the 2D algorithm and two more classes a Global Planner class and a Local Planner class that uses the Lazy Theta* class algorithm. 

## Installation

You need the full ROS Kinetic installation. Follow the steps in http://wiki.ros.org/kinetic/Installation/Ubuntu and choose 

```
sudo apt-get install ros-kinetic-desktop-full
```

Once ROS is installed, inside your catkin workspace, src folder:

```
git clone https://github.com/robotics-upo/theta_star_2d.git
```

Compile the package:

```
cd catkin_ws/ && catkin_make
```
Also check that costmap_2d package and map_server are installed.
```
sudo apt-get install ros-kinetic-map-server ros-kinetic-costmap-2d
```

If you want to use social layers for costmap 2d, you have to install them:

```
sudo apt-get install ros-kinetic-social-navigation-layers
```

Finally, ThetaStar class uses boost string algorithms, so you need the Boost C++ Libraries: https://www.boost.org/


## Nodes
- **Global Planner Node**: Input: map or costmap(occupancy grid in both cases). It expects a PoseStamped Goal. Once it has calculated the path, it waits for another goal request(another goal message published) or a replanning service call.
  
    In this new version the global costmap is integrated in the global planner with the costmap wrapper, so you have also to load a global costmap configuration parameter file, like the example one located in the cfg/ folder.

- [ ] TODO: Make it work as a service.
- [ ] TODO: Use action lib for long time execution servies. 

    ### Parameters
  - robot_base_frame: usually /base_link frame.
  - world_frame: usually /map frame.
  - show_config: Boolean to print out the config received by the node on the terminal
  - debug: Boolean to enable debug messages
  - traj_dxy_max: Max separation between points of the output trajectory
  - traj_pos_tol: Position tolerance of the output trajectory
  - traj_yaw_tol: Yaw tolerance of the output trajectory

    ### Services
    - `/global_planner_node/global_replanning_service`:Call this service when you want a new path to the same previous goal. Type: std_srvs::Trigger
    - `/global_planner_node/reset_costmap`: Reset costmap layers. Type: std_srvs::Trigger.
  
 The subscribed and published topics are also configured as parameters. Neverthless it's recommended to use the default names because there are other nodes (mainly the local planner node and the arco path tracker in the case you are also using it) depending on them.
    
- Subscriptions topics name parameters:
    
    - goal_topic: The geometry_msgs::PoseStamped goal topic, by default /move_base_simple/goal in order to easily command goals from RViz
    
- Publishing topics name parameters
    - vis_marker_traj_topic: The topic where will be published the visualization_msgs::Marker to visualize the trajectory on RViz. Not related to any other node.
    - traj_topic: By default /trajectory_tracker/input_trajectory. It's where the nav_msgs::MultiDOFJointTrajectory will be published. Topic subscribed by local planner


- **Custom Costmap Node**: It's a standar costmap node but with an additional service server to reset the layers. It's though to be used with the local planner.
  ### Services
    - `/reset_costmap`: Service of type std_srvs::Trigger to reset costmap layers.
  
- [ ] TODO: Include this costmap in the local planner as done in the global planner node. Maybe I'll need to use two threads to simultaneously update costmap and trajectories.

- **Local Planner Node**: This nodes receives the path from the global planner node and a local costmap centered in the robot. In this new version you can use the custom costmap node located in the package. It tries to follow the global path accounting for dynamic obstacles. It needs a local costmap (tipical rollin window costmap) to work. 

    ### Parameters 

  - robot_base_frame: usually /base_link frame.
  - world_frame: usually /map frame.
  - local_costmap_infl_x and _y: The size of the border of the occupied space added to the costmap in x and y direction. It depends for example on how long are your global path points separated one from another.
  - border space: The space that will be free around in the local goal in the border mentioned above.
  - traj_dxy_max: Max separation between points of the output trajectory
  - traj_pos_tol: Position tolerance of the output trajectory
  - traj_yaw_tol: Yaw tolerance of the output trajectory
  - show_config: Boolean to see the config received at startup by the node on the terminal
  - debug: Boolean to enable debug messages
  - show_config: Boolean to print out on the terminal the configuration readed by the node at startup. 

For a more detailed description, see the inflateLocalCostmap and freeLocalGoal functions in LocalPlanner class. The subscribed and published topics are also configured as parameters. Neverthless it's recommended to use the default names because there are other nodes depending on them.

- Services Clients: 
  - `/global_planner_node/global_replanning_srv`
  - `/custom_costmap_node/reset_costmap`
  
- Services Servers:
  - `/local_planner_node/stop_planning_srv`: std_srvs::Trigger
  - `/local_planner_node/pause_planning_srv`: std_srvs::Trigger
  
- Subsribed topics:
   
  - local_costmap_topic: Usually /custom_costmap_node/costmap/costmap if you use the include costmap node with reset layer service included in the package.
  - global_traj_topic: The trajectory topic published by the **global planner**
    
- Published topics name paramters

  - local_traj_output_topic: The trajectory published and readed by the navigation. Usually /trajectory_tracker/local_input_trajectory
  - local_trajectory_markers_topic: The topic where the RViz markers will be published. Usually /local_planner_node/visualization_marker_trajectory
  - local_planning_time: Topic to monitorize the time spent in calculating trajectories as std_msgs::Int32 (miliseconds).
  - inf_costmap_pub: Not really used, only for debugging purposes. The node will publish the modified costmap, with the extra occupied borders and the free space around the local goal. Usually /local_costmap_inflated, but there are no nodes depending on it. 
  - running_state_pub: Communication flag. Usuallly /local_planner_node/running
  - occ_goal_pub: Communication flag. Usually /trajectory_tracker/local_goal_occupied
  - impossible_to_find_sol_pub: Communication flag. Usually /trajectory_tracker/impossible_to_find

- **Sim Planner Node**: It's basically a global planner adapted to run almost isolated. You only need to launch a map or costmap from map_server or costmap_2d package. Using RViz you can command start and goal positions. There also hardcoded parameters to test the algorithms with varying parameters. 
  - With the new versions of the planners this node is deprecated.

### Dynamically reconfigurable parameters common to all nodes


- Goal weight: The goal weight parameter of the heuristic $h(n) = g_w * dist(n,goal)$
- Cost weight: This parameter quantifies the effect of the cost of the costmap. Usefull values are (0,0.4]. 
- Line of sight: The restriction of the line of sight imposed to the algorithm in order to make work the cost-sensible modification. This value depends on the geometry of the scenario. For example, if you restrict the line of sight to 5 meters in an small scenario with distances minors of 5m, the cost won't have any effect. You must tune it depending on the scenario.
- Occupation Threshold: This is the value whereupon the algorithm will set the nodes to occupied. 


If you find some parameter that will be usefull to add as reconfigurable parameters let me know!
