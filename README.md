# Lazy Theta* 2D 

In this repository you can find the library of the Lazy Theta* with Optimizations and Cost Adapted of the 2d algorithm. There also 3 ROS Nodes:

## Nodes
- Global Planner Node: Input: map or costmap, you must specify the topic the code (Todo:Change to parameter). It expects a PoseStamped Goal from the topic /move_base_simple/goal. Once it calculated the path, it waits for another goal request.
  
- Local Planner Node: This nodes receives the path from the global planner node and a local costmap centered in the robot. It tries to follow the global path accounting for dynamic obstacles. 
  
- Sim Planner Node: It's basically a global planner node adapted to run almost isolated. You only need to launch a map or costmap from map_server or costmap_2d package. Using RViz you can command start and goal positions. 

## Parameters

Static parameters you must include in a launch such the example launch in the launch folder:

- ws_x_max: Workspace max x coordinate: The maximum x coordinate of the map (meters). It depends on the resolution of the map and the width in pixels. For a 400 pixel wide map and 0.05 m/pixel, $ws_x_max = 400*0.05 = 20$.
- ws_y_max: Workspace max y coordinate: Same as the ws_x_max parameter. 
- ws_x_min: Workspace min x coordinate: The min x, you can set it to zero.
- ws_y_min: Workspace min y coordinate: The min y, you can set it to zero.
- map_resolution: The resolution of the occupancy grid used. It depends usually on the gmapping settings you used. 


Also there are four dynamically reconfigurable parameters:

- Goal weight: The goal weight parameter of the heuristic $h(n) = g_w * dist(n,goal)$
- Cost weight: This parameter quantifies the effect of the cost of the costmap. Usefull values are (0,0.4]. 
- Line of sight: The restriction of the line of sight imposed to the algorithm in order to make work the cost-sensible modification. This value depends on the geometry of the scenario. For example, if you restrict the line of sight to 5 meters in an small scenario with distances minors of 5m, the cost won't have any effect. You must tune it depending on the scenario.
- Occupation Threshold: This is the value whereupon the algorithm will set the nodes to occupied. 

## Todo list

- [] Integrate the geometry params in the algorithm to get them directly from the topic
- [] Configure topics as params
