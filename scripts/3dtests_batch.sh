#!/bin/bash


source /opt/ros/$(rosversion -d)/setup.bash

lof="1 2 3" 
cost_w="0.1 0.2 0.3"
timeout=350
map=$(rospack find theta_star_2d) 
mkdir -p ~/3dtest/

initial_point="1 1 1"
initial_array=($initial_point)
goal_point="1 1 1" 
goal_array=($goal_point)
echo "${goal_array[0]}"

for line in $lof; do
    for cost in $cost_w; do
        echo "cost: $cost, line of sight: $line, timeout: $timeout , map: $map"
        #Launch here the stuff
        # roslaunch theta_star_2d test_3d_costmap.launch line_of_sight:=$line cost_weight:=$cost timeout:=$timeout map:=$map
        
        #Call service blocking to get map and stuff
        rosservice call --wait /global_planner_node/compute_path 
        test_name="$map"_lof_"$line"_cw_"$cost"
        echo "$test_name"
        #Gnome-screenshot
        # gnome-screenshot -f ~/3dtest/$test_name.png
        #kill entire launch
        rosnode kill /global_planner_node
        sleep 5
        spd-say 'Test done'
    done
done

spd-say 'All tests finished'

