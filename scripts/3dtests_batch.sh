#!/bin/bash

source /opt/ros/$(rosversion -d)/setup.bash
source $1/devel/setup.bash

lof=$2
cost_w=$3
map=$4
initial_point="102 106 3"
goal_point="103 106 3" 
n_tries=$5
timeout=350

map_path=$(rospack find theta_star_2d)/$map
mkdir -p ~/3dtest/

initial_array=($initial_point)
goal_array=($goal_point)

roslaunch theta_star_2d test_3d_costmap.launch timeout:=$timeout batch_mode:=true map:=$map_path & 
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
          rosservice call --wait /global_planner_node/get_path "start:
  x: ${initial_array[0]}
  y: ${initial_array[1]}
  z: ${initial_array[2]}
goal:
  x: ${goal_array[0]}
  y: ${goal_array[1]}
  z: ${goal_array[2]}" >> ~/3dtest/"$map"_lof_"$line"_cw_"$cost"_$n.txt

          test_name="$map"_lof_"$line"_cw_"$cost"_$n.png
          echo "$test_name"
          #Gnome-screenshot
          gnome-screenshot -f ~/3dtest/$test_name
          spd-say 'Test done'
      done
    done
done

spd-say 'All tests finished'

