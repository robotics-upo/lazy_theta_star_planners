#!/bin/bash

#You need to tell the script the original .bt, and the parameters robot radius and cost 
#scaling factor ranges
#usage: rosrun theta_star_2d generate_bunch_of_gridm.sh <path_to_workspace> <map_to_maps_folder> <map_name_with bt extension> <robot radius values list> <cost scaling factors values list>

source /opt/ros/$(rosversion -d)/setup.bash
source $1/devel/setup.bash
maps_folder=$2
map_file=$3
map_name=$(basename $map_file .bt)
robot_radius_values=$4
cost_scaling_factors_values=$5
echo "Generating gridm files for"
echo "Cost Factors: $cost_scaling_factors_values"
echo "Robot Radius: $robot_radius_values"
read -p "Press enter to continue"
roscore & &>/dev/null
sleep 3
for robot_radius in $robot_radius_values; do
   for cost_factor in $cost_scaling_factors_values; do
       echo -e "\e[32m Generating .gridm for RR: $robot_radius and Cost Factor $cost_factor \e[0m"
       rosrun theta_star_2d grid3d_generator $maps_folder/$map_file _use_costmap_function:=true _cost_scaling_factor:=$cost_factor _robot_radius:=$robot_radius
       mv "$maps_folder/$map_name".gridm $maps_folder/"$map_name"_rr_"$robot_radius"_csf_"$cost_factor".gridm
   done    
done
killall rosmaster