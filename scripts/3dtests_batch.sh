#!/bin/bash

source /opt/ros/$(rosversion -d)/setup.bash
source $1/devel/setup.bash

lof=$2
cost_w=$3
map=$4
initial_point="13 13 3"
goal_point="15 15 5" 
n_tries=$5
timeout=350
#THIS PARAM SHOULD BE CHANGE
robot_radius=1
map_path=$(rospack find theta_star_2d)/maps/$map
map_list=$(ls $(rospack find theta_star_2d)/maps/ | grep .gridm)
bt_file=$(ls $(rospack find theta_star_2d)/maps/ | grep .bt)

mkdir -p ~/3dtest/$map

initial_array=($initial_point)
goal_array=($goal_point)

bt_file_name=$(basename $bt_file .bt)
cd $(rospack find theta_star_2d)/maps/
for grid in $map_list; do
  echo "Using gridm file: $grid"
  mv $grid $bt_file_name.gridm
  roslaunch theta_star_2d test_3d_costmap.launch timeout:=$timeout batch_mode:=true map:=$map_path & 
  sleep 10
  #Set the cost values automatically
  rosservice call /theta_star_3d/set_costs_values "robot_radius: $robot_radius
cost_scaling_factor: $cost_w"
  sleep 30
  for line in $lof; do
     for cost in $cost_w; do
       for (( n=1; n<=$n_tries; n++ ))
       do
           echo "Try number $n / $n_tries"
           echo "cost: $cost, line of sight: $line, timeout: $timeout , map: $map"
           #Launch here the stuff
           rosrun dynamic_reconfigure dynparam set /global_planner_node goal_weight $cost
           rosrun dynamic_reconfigure dynparam set /global_planner_node lof_distance $line
           #Call service blocking to get map and stuff
           rostopic echo /theta_star_3d/closest_distance -n 1 >> ~/3dtest/$map/min_distance_"$map"_lof_"$line"_cw_"$cost"_$n.txt &
           rosservice call --wait /global_planner_node/get_path "start:
   x: ${initial_array[0]}
   y: ${initial_array[1]}
   z: ${initial_array[2]}
goal:
   x: ${goal_array[0]}
   y: ${goal_array[1]}
   z: ${goal_array[2]}" >> ~/3dtest/$map/"$map"_lof_"$line"_cw_"$cost"_$n.txt

           test_name="$map"_lof_"$line"_cw_"$cost"_$n.png
           echo "$test_name"
           #Gnome-screenshot
           gnome-screenshot -f ~/3dtest/$map/$test_name
           spd-say 'Test done'
       done
     done
  done
  rosnode kill /global_planner_node
  sleep 5
  mv $bt_file_name.gridm $grid
done
spd-say 'All tests finished'

