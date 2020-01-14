/*
Rafael Rey Arcenegui, 2019 UPO

Global Planner Class using the Lazy ThetaStar 2d Algorithm
*/
#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_

#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <memory>
#include <theta_star/ThetaStar2D.hpp>
#include <theta_star/ThetaStar3D.hpp>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/Bool.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Dynamic reconfigure auto generated libraries
#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/GlobalPlannerConfig.h>

#include <ctime>
#include <sys/timeb.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <upo_actions/ExecutePathAction.h>
#include <upo_actions/MakePlanAction.h>
#include <upo_actions/RotationInPlaceAction.h>


#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <pcl_ros/point_cloud.h>

namespace PathPlanners
{
class GlobalPlanner : public ThetaStar3D
{

    typedef actionlib::SimpleActionClient<upo_actions::ExecutePathAction> ExecutePathClient;
    typedef actionlib::SimpleActionServer<upo_actions::MakePlanAction> MakePlanServer;
    typedef actionlib::SimpleActionClient<upo_actions::RotationInPlaceAction> RotationInPlaceClient;

public:
    //Default constructor
    GlobalPlanner(string node_name);
    /**
		Default destructor
	**/
    // ~GlobalPlanner();

    /*
    @brief: This is the main function that should be executed in loop by the node
    */
    void plan();

    /* 
    @brief: Dynamic reconfiguration server callback declaration
    */
    void dynReconfCb(theta_star_2d::GlobalPlannerConfig &config, uint32_t level);
   
private:
    void clearMarkers();
    void sendPathToLocalPlannerServer();

    //Action server
    void makePlanPreemptCB();
    void makePlanGoalCB();
    void publishMakePlanFeedback();
    int getClosestWaypoint();
    bool replan();
    
    /*
    @brief: Calls the resetLayers() costmap_2d member function in order to clean old obstacles 
    */
    bool resetCostmapSrvCb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &rep);
    /*
    @brief: Loads parameters from ros param server, if they are not present, load defaults ones
            It also configure markers and global map geometry 
    */
    void configParams2D();
    void configParams3D();
    /*
    @brief: Load topics names from param server and if they are not present, set defaults topics names for 
            Subscribers and publishers
    */
    void configTopics();
    /*
    @brief: Config thetastar class parameters
    */
    void configTheta();
    /*
    @brief: It declares the two service servers
    */
    void configServices();

    //get robot pose to know from where to plan
    geometry_msgs::TransformStamped getRobotPose();
    /*
    @brief: Lock and reset costmap layers
    */
    void resetGlobalCostmap();
    void collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
    void publishTrajectory2D();
    void publishTrajectory3D();


    bool calculatePath();

    /*
    @brief: These functions tries to pass the start and goal positions to the thetastar object
            They return true if the points are not occupied
    */
    bool setGoal();
    bool setStart();
    void calculatePathLength();
    /*              Class Variables                 */
    ros::NodeHandlePtr nh;

    //!New Markers(Line strip + waypoints)
    visualization_msgs::Marker lineMarker, waypointsMarker;

    geometry_msgs::PoseStamped goalPoseStamped;
    geometry_msgs::Vector3Stamped goal;

    //Publishers and Subscribers
    ros::Publisher replan_status_pub,visMarkersPublisher;
    ros::Subscriber goal_sub, sub_map;

    //Services servers
    ros::ServiceServer global_replanning_service, reset_global_costmap_service, plan_request_service;
    ros::ServiceClient recovery_rot_srv_client;
    //ThetaStar object
    ThetaStar2D theta2D;

    //tf buffer used to get the base_link position on the map(i.e. tf base_link-map)
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_list;

    std::unique_ptr<tf::TransformListener> tf_list_ptr;
    std::unique_ptr<costmap_2d::Costmap2DROS> global_costmap_ptr;

    std_msgs::Bool flg_replan_status;

    float cost_weight;
    float occ_threshold;
    float lof_distance;

    string robot_base_frame, world_frame, node_name;

    //Output variables
    int number_of_points;
    int nbrRotationsExec;
    int seq;
    float pathLength;
    Trajectory trajectory;

    //These two flags can be configured as parameters
    bool showConfig, debug;

    int countImpossible = 0;

    //Action client stuff
    std::unique_ptr<ExecutePathClient> execute_path_client_ptr;

    std::unique_ptr<MakePlanServer> make_plan_server_ptr;
    upo_actions::MakePlanFeedback make_plan_fb;
    upo_actions::MakePlanResult make_plan_res;

    //
    std::unique_ptr<RotationInPlaceClient> rot_in_place_client_ptr;
    upo_actions::RotationInPlaceGoal rot_in_place_goal;

    //
    //Vairables to fill up the result MakePlan result field
    std_msgs::UInt8 emergencyStopNbr;
    std_msgs::Duration  time_spent;
    //Variables to fill up the feedback 
    std_msgs::Float32 dist2Goal;
    std_msgs::Duration travel_time;
    std_msgs::String percent_achieved, ETA;
    std_msgs::UInt8 globalWaypoint;
    int timesReplaned;
    struct timeb start, finish;
    float seconds, milliseconds;
    float minPathLenght;
    ros::Time start_time;

    //! 3D specific variables
    ThetaStar3D theta3D;
    bool use3d;
    bool data_source;

    octomap_msgs::OctomapConstPtr map;

    double ws_x_max; // 32.2
    double ws_y_max; // 32.2
    double ws_z_max;
    double ws_x_min;
    double ws_y_min;
    double ws_z_min;
    double map_resolution;
    double map_h_inflaction;
    double map_v_inflaction; //JAC: Hasta aquí todo cero.
    double goal_weight;
    double z_weight_cost;
    double z_not_inflate;
    double traj_dxy_max;
    double traj_dz_max;
    double traj_vxy_m;
    double traj_vz_m;
    double traj_vxy_m_1;
    double traj_vz_m_1;
    double traj_wyaw_m;
    double traj_pos_tol;
    double traj_yaw_tol;
}; //class GlobalPlanner

} //namespace PathPlanners

#endif