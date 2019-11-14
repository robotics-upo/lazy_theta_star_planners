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

#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/LocalPlannerConfig.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <upo_actions/ExecutePathAction.h>
#include <upo_actions/NavigateAction.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

namespace PathPlanners
{
class LocalPlanner : public ThetaStar
{
    typedef actionlib::SimpleActionServer<upo_actions::ExecutePathAction> ExecutePathServer;
    typedef actionlib::SimpleActionClient<upo_actions::NavigateAction> NavigateClient;

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
    void executePathGoalServerCB();
    void executePathPreemptCB();
    void dist2GoalCb(const std_msgs::Float32ConstPtr &dist);

private:
    void resetFlags();
    void clearMarkers();
    //These functions gets parameters from param server at startup if they exists, if not it passes default values
    void configParams();
    void configTopics();
    //Config thetastar class parameters
    void configTheta();
    void configServices();

    void publishExecutePathFeedback();

    bool calculateLocalGoal();

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
    ros::NodeHandlePtr nh;
    ros::ServiceClient costmap_clean_srv;
    ros::Subscriber local_map_sub;
    ros::Publisher running_state_pub,visMarkersPublisher, trajPub;

    //Flags publishers

    bool showConfig;
    bool mapGeometryConfigured;
    bool doPlan, First;
    //Flow control flags
    bool localCostMapReceived;
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
    int impossibleCnt, occGoalCnt, timesCleaned, badGoal;
    
    int number_of_points;
    bool debug;
    float seconds, milliseconds;

    string robot_base_frame, world_frame;
    nav_msgs::OccupancyGrid localCostMap, localCostMapInflated;
    trajectory_msgs::MultiDOFJointTrajectory globalTrajectory, localTrajectory;
    //Markers
    visualization_msgs::Marker lineMarker, waypointsMarker;

    geometry_msgs::Vector3 local_costmap_center, localGoal;
    ThetaStar lcPlanner;
    tf2_ros::Buffer *tfBuffer;

    std_msgs::Bool is_running;

    //action server stufff
    std::unique_ptr<ExecutePathServer> execute_path_srv_ptr;

    upo_actions::ExecutePathFeedback exec_path_fb;
    std_msgs::Float32 planningRate;
    std_msgs::UInt8 waypointGoingTo;
    std_msgs::String planningStatus;

    upo_actions::ExecutePathResult action_result;

    ros::Time start_time;
    std_msgs::Float32 d2goal;
    //action client to navigate
    std::unique_ptr<NavigateClient> navigate_client_ptr;
    upo_actions::NavigateGoal nav_goal;

};

} // namespace PathPlanners

#endif