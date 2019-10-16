/*





 */
#ifndef LOCALPLANNER_H_
#define LOCALPLANNER_H_

#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>
#include <fstream>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

#include <geometry_msgs/PoseStamped.h>

#include <theta_star/ThetaStar.hpp>

#include <visualization_msgs/MarkerArray.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/LocalPlannerConfig.h>


#include <actionlib/server/simple_action_server.h>
#include <theta_star_2d/ExecutePathAction.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
namespace PathPlanners
{
class LocalPlanner : public ThetaStar
{
typedef actionlib::SimpleActionServer<theta_star_2d::ExecutePathAction> ActionServer;
public:
    /*
        Default constructor    
    */
    LocalPlanner(tf2_ros::Buffer *tfBuffer_);

    /*
    
    */
    void plan();

    //Calbacks and publication functions
    void localCostMapCb(const nav_msgs::OccupancyGrid::ConstPtr &lcp);
    //Global Input trajectory
    void globalTrjCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &traj);
    
    //Used to know when to stop calculating local trajectories
    void goalReachedCb(const std_msgs::Bool::ConstPtr &data);

    void dynRecCb(theta_star_2d::LocalPlannerConfig &config, uint32_t level);
    bool stopPlanningSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep);
    bool pausePlanningSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep);
  

    bool arrivedToGoalSrvCb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp);
    void actionGoalServerCB();
    void actionPreemptCB();
    void dist2GoalCb(const std_msgs::Float32ConstPtr &dist);
private:
    //These functions gets parameters from param server at startup if they exists, if not it passes default values
    void configParams();
    void configTopics();
    //Config thetastar class parameters
    void configTheta();
    void configServices();

    geometry_msgs::Vector3 calculateLocalGoal();

    geometry_msgs::TransformStamped getTfMapToRobot();

    void publishTrajMarker();

    void buildAndPubTrayectory();

    //Auxiliar functions
    void showTime(string message, struct timeb st, struct timeb ft);

    //Add the occupied borders around received costmap
    void inflateCostMap();
    //Set to 0 the positions near the local goal in the border
    void freeLocalGoal();

    //Variables
    ros::NodeHandle nh_;
    ros::ServiceClient replanning_client_srv, costmap_clean_srv, stop_nav_client_srv;
    ros::ServiceServer stop_planning_srv,pause_planning_srv, arrived_to_goal_srv;
    ros::Subscriber local_map_sub, goal_reached_sub, global_goal_sub, global_trj_sub, dist2goal_sub;
    //TODO: Replace global goal publisher used to request a new global trajectory by a Service call
    //Not much sense that local planner publishes global goals
    ros::Publisher trajectory_pub, vis_marker_traj_pub, global_goal_pub, local_planning_time,inf_costmap_pub;

    //Flags publishers
    ros::Publisher running_state_pub, occ_goal_pub, impossible_to_find_sol_pub;

    bool showConfig;
    bool mapGeometryConfigured;
    bool doPlan;
    //Flow control flags
    bool localCostMapReceived;
    bool globalTrajReceived;
    //To calculate planning time
    struct timeb startT, finishT;

    //Theta star algorithm parameters
    //Geometric params
    float map_resolution;
    float ws_x_max;
    float ws_y_max;
    //Algorithm params
    float goal_weight;
    float cost_weight;
    float lof_distance;
    float occ_threshold;
    //Traj params
    float traj_dxy_max;
    float traj_pos_tol;
    float traj_yaw_tol;
    //Costmap modificaton params
    float localCostMapInflationX;
    float localCostMapInflationY;
    float border_space;

    //

    unsigned int startIter;

    string robot_base_frame, world_frame;

    nav_msgs::OccupancyGrid localCostMap, localCostMapInflated;

    trajectory_msgs::MultiDOFJointTrajectory globalTrajectory, localTrajectory;
    
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerTraj;
    
    geometry_msgs::Vector3 local_costmap_center, localGoal;
   
    ThetaStar lcPlanner;

    tf2_ros::Buffer *tfBuffer;

    std_msgs::Bool is_running, occ, impossible_calculate;
    std_msgs::Int32 time_spent_msg;
    
    int impossibleCnt,occGoalCnt;
    int number_of_points;
    bool startOk;
    bool debug;
    float seconds, milliseconds;
    //action server stufff
    std::unique_ptr<actionlib::SimpleActionServer<theta_star_2d::ExecutePathAction>> plan_server_ptr;
    theta_star_2d::ExecutePathFeedback action_feedback;
    theta_star_2d::ExecutePathResult action_result;

    ros::Time start_time;
    std_msgs::Float32 d2goal;
};

} // namespace PathPlanners

#endif