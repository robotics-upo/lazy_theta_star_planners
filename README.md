# Lazy Theta* 2D 

In this repository you can find the library of the Lazy Theta* with Optimizations and Cost Adapted of the 2d algorithm and two more classes a Global Planner class and a Local Planner class that uses the Lazy Theta* class algorithm. 

## Installation

You need the full ROS Kinetic installation. Follow the steps in http://wiki.ros.org/kinetic/Installation/Ubuntu and choose 

```
sudo apt-get install ros-kinetic-desktop-full
```

Once ROS is installed, inside your catkin workspace, src folder:

```
git clone https://github.com/robotics-upo/theta_star_2d.git
```

Return to the catkin workspace base path and do

```
catkin_make
```
Also check that costmap_2d package and map_server are installed.
```
sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-costmap-2d
```

If you want to use social layers for costmap 2d, you have to install them:

```
sudo apt-get install ros-kinetic-social-navigation-layers
```

Finally, ThetaStar class uses boost string algorithms, so you need the Boost C++ Libraries: https://www.boost.org/


## Nodes
- **Global Planner Node**: Input: map or costmap(occupancy grid in both cases). It expects a PoseStamped Goal. Once it calculated the path, it waits for another goal request(another goal message published).
- [ ] TODO: Make it work as a service. 

    ### Parameters
  - ws_x_max: Workspace max x coordinate: The maximum x coordinate of the map (meters). It depends on the resolution of the map and the width in pixels. For a 400 pixel wide map and 0.05 m/pixel, $ws_x_max = 400*0.05 = 20$.
  - ws_y_max: Workspace max y coordinate: Same as the ws_x_max parameter. 
  - map_resolution: The resolution of the occupancy grid used. It depends usually on the gmapping settings you used. 
  - robot_base_frame: usually /base_link frame.
  - world_frame: usually /map frame.
  - show_config: Boolean to see at startup  the config received by the node on the terminal
  - debug: Boolean to enable debug messages
  - traj_dxy_max: Max separation between points of the output trajectory
  - traj_pos_tol: Position tolerance of the output trajectory
  - traj_yaw_tol: Yaw tolerance of the output trajectory

 The subscribed and published topics are also configured as parameters. Neverthless it's recommended to use the default names because there are other nodes depending on them
    
- Subscriptions topics name parameters:
    
    - global_map_topic: It could be a costmap or a pgm map from map_server(nav_msgs::OccupancyGrid).By default /costmap_2d/costmap/costmap
    - goal_topic: The geometry_msgs::PoseStamped goal topic, by default /move_base_simple/goal in order to easily command goals from RViz
    
- Publishing topics name parameters
    - vis_marker_traj_topic: The topic where will be published the visualization_msgs::Marker to visualize the trajectory on RViz. Not related to any other node.
    - traj_topic: By default /trajectory_tracker/input_trajectory. It's where the nav_msgs::MultiDOFJointTrajectory will be published. Topic subscribed by local planner


- **Local Planner Node**: This nodes receives the path from the global planner node and a local costmap centered in the robot. It tries to follow the global path accounting for dynamic obstacles. It needs a local costmap (tipical rollin window costmap) to work. 

    ### Parameters 


  - ws_x_max: Workspace max x coordinate: The maximum x coordinate of the map (meters). It depends on the resolution of the map and the width in pixels. For a 400 pixel wide map and 0.05 m/pixel, $ws_x_max = 400*0.05 = 20$.
  - ws_y_max: Workspace max y coordinate: Same as the ws_x_max parameter. 
  - map_resolution: The resolution of the occupancy grid used. It depends usually on the gmapping settings you used. 
  - robot_base_frame: usually /base_link frame.
  - world_frame: usually /map frame.
  - local_costmap_infl_x and _y: The size of the border of the occupied space added to the costmap in x and y direction. It depends for example on how long are your global path points separated one from another.
  - border space: The space that will be free around in the local goal in the border mentioned above.
  - traj_dxy_max: Max separation between points of the output trajectory
  - traj_pos_tol: Position tolerance of the output trajectory
  - traj_yaw_tol: Yaw tolerance of the output trajectory
  - show_config: Boolean to see the config received at startup by the node on the terminal
  - debug: Boolean to enable debug messages

For a more detailed description, see the inflateLocalCostmap and freeLocalGoal functions in LocalPlanner class. The subscribed and published topics are also configured as parameters. Neverthless it's recommended to use the default names because there are other nodes depending on them.

- Subsribed topics:
   
  - local_costmap_topic: Usually /costmap_2d_local/costmap/costmap. Not related to any other node
  - goal_reched_topic: Flag status published by the navigation to stop calculating local paths once goal is reached.
  - global_goal_topic: It should be the same as in the global planner. Usually /move_base_simple/goal for simplicity.
  - global_traj_topic: The trajectory topic published by the **global planner**
    
- Published topics name paramters

  - local_traj_output_topic: The trajectory published and readed by the navigation. Usually /trajectory_tracker/local_input_trajectory
  - local_trajectory_markers_topic: The topic where the RViz markers will be published. Usually /local_planner_node/visualization_marker_trajectory
  - global_goal_pub: The same as before, usually /move_base_simple/goal. Used to republish the current goal to request a global replanning. 
  - local_planning_time: Topic to monitorize the time spent in calculating trajectories as std_msgs::Int32 (miliseconds).
  - inf_costmap_pub: Not really used, only for debugging purposes. The node will publish the modified costmap, with the extra occupied borders and the free space around the local goal. Usually /local_costmap_inflated, but there are no nodes depending on it. 
  - running_state_pub: Communication flag. Usuallly /local_planner_node/running
  - occ_goal_pub: Communication flag. Usually /trajectory_tracker/local_goal_occupied
  - impossible_to_find_sol_pub: Communication flag. Usually /trajectory_tracker/impossible_to_find

- **Sim Planner Node**: It's basically a global planner node adapted to run almost isolated. You only need to launch a map or costmap from map_server or costmap_2d package. Using RViz you can command start and goal positions. There also hardcoded parameters to test the algorithms with varying parameters

There are also some parameters specific of the local planner node:


### Dynamically reconfigurable parameters common to all nodes


- Goal weight: The goal weight parameter of the heuristic $h(n) = g_w * dist(n,goal)$
- Cost weight: This parameter quantifies the effect of the cost of the costmap. Usefull values are (0,0.4]. 
- Line of sight: The restriction of the line of sight imposed to the algorithm in order to make work the cost-sensible modification. This value depends on the geometry of the scenario. For example, if you restrict the line of sight to 5 meters in an small scenario with distances minors of 5m, the cost won't have any effect. You must tune it depending on the scenario.
- Occupation Threshold: This is the value whereupon the algorithm will set the nodes to occupied. 



If you find some parameter that will be usefull to add as reconfigurable parameters let me know!

## Todo list

- [ ] Integrate the geometry params in the algorithm to get them directly from the topic
- [x] Configure topics as params
