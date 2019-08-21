/*





 */
#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_

#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <ros/ros.h>

#include <theta_star/ThetaStar.hpp>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Dynamic reconfigure auto generated libraries
#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/globalPlannerConfig.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace PathPlanners
{
class GlobalPlanner : public ThetaStar
{

public:
    //Default constructor
    GlobalPlanner(tf2_ros::Buffer *tfBuffer_, string node_name);
    /**
		Default destructor
	**/
	// ~GlobalPlanner();

    //Main function
    void plan();
    
    //Dynamic callback
    void dynReconfCb(theta_star_2d::globalPlannerConfig &config, uint32_t level); 

private:
    
    void goalCb(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);

    bool replanningSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep);
    bool resetCostmapSrvCb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &rep);
    
    //These functions gets parameters from param server at startup if they exists, if not it passes default values
    void configParams();
    void configTopics();
    //Config thetastar class parameters
    void configTheta();
    void configServices();
    
    //get robot pose to know from where to plan
    geometry_msgs::TransformStamped getRobotPose();
    void resetGlobalCostmap();
    void publishTrajectory();

    bool calculatePath();
    bool setGoal();
    bool setStart();

    //Input variables
    ros::NodeHandle nh_;

    visualization_msgs::MarkerArray markerTraj;
    visualization_msgs::Marker marker;
    
    geometry_msgs::PoseStamped goalPoseStamped;
    geometry_msgs::Vector3Stamped goal;

    //Input parameters
    float map_resolution;
    float ws_x_max;
    float ws_y_max;

    float cost_weight;
    float goal_weight;
    float occ_threshold;
    float lof_distance;
    
    float traj_dxy_max;
    float traj_pos_tol;
    float traj_yaw_tol;

    string robot_base_frame, world_frame,node_name;

    //Output variables
    int number_of_points;
    Trajectory trajectory;
    //Control flags

    bool globalGoalReceived;
    bool showConfig,debug;
    
    //Publishers and Subscribers
    ros::Publisher trj_pub, vis_trj_pub;
    ros::Subscriber goal_sub, global_costmap_sub;

    ros::ServiceServer global_replanning_service,reset_global_costmap_service;
    //tf buffer used to get the base_link position on the map(i.e. tf base_link-map)
    tf2_ros::Buffer *tfBuffer;
    //ThetaStar object
    ThetaStar gbPlanner;

    tf::TransformListener *tf_list_ptr;
    costmap_2d::Costmap2DROS *global_costmap_ptr;

}; //class GlobalPlanner

} //namespace PathPlanners

#endif