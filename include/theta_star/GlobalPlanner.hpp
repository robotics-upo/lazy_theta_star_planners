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

#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Dynamic reconfigure auto generated libraries
#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/globalPlannerConfig.h>

namespace PathPlanners
{
class GlobalPlanner : public ThetaStar
{

public:
    //Default constructor
    GlobalPlanner(tf2_ros::Buffer *tfBuffer_);
    /**
		Default destructor
	**/
	// ~GlobalPlanner();

    //Main function
    void plan();
    
    //Callbacks
    void dynReconfCb(theta_star_2d::globalPlannerConfig &config, uint32_t level);
    void globalCostMapCb(const nav_msgs::OccupancyGrid::ConstPtr &fp);
    void goalCb(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
   
private:

    //These functions gets parameters from param server at startup if they exists, if not it passes default values
    void configParams();
    void configTopics();
    //Config thetastar class parameters
    void configTheta();
    
    //get robot pose to know from where to plan
    geometry_msgs::TransformStamped getRobotPose();
    
    void publishTrajectory();

    bool setGoal();
    bool setStart();


    //Input variables

    ros::NodeHandle nh_;

    visualization_msgs::Marker markerTraj;

    geometry_msgs::PoseStamped goalPoseStamped;
    geometry_msgs::Vector3Stamped goal;

    nav_msgs::OccupancyGrid globalCostMap;

    //Input parameters

    float map_resolution;
    float ws_x_max;
    float ws_y_max;
    float ws_x_min;
    float ws_y_min;
    float cost_weight;
    float goal_weight;
    float occ_threshold;
    float lof_distance;
    float traj_dxy_max;
    float traj_pos_tol;
    float traj_yaw_tol;

    string robot_base_frame, world_frame;
    //Output variables
    int number_of_points;
    Trajectory trajectory;
    //Control flags

    bool globalGoalReceived;
    bool gCmReceived;

    bool showConfig,debug,mapParamsConfigured;
    
    //Publishers and Subscribers
    ros::Publisher trj_pub, vis_trj_pub;
    ros::Subscriber goal_sub, global_costmap_sub;
    //tf buffer used to get the base_link position on the map(i.e. tf base_link-map)
    tf2_ros::Buffer *tfBuffer;
    //ThetaStar object
    ThetaStar gbPlanner;
}; //class GlobalPlanner

} //namespace PathPlanners

#endif