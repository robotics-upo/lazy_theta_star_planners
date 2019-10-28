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
#include <theta_star/ThetaStar.hpp>

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

struct ReportElement
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory;
    geometry_msgs::PoseStamped goal, robot_pose;
    ros::Time requested, published;
    bool replan;
};

class MissionReport
{
public:
    MissionReport()
    {
        data.resize(0);
    }
    void newElement()
    {
        ReportElement elem;
        data.push_back(elem);
    }
    /*
    Puedo crear una funcion publica que se llame con cada nueva trayectoria creada
    *Se guarda la trayectoria con todos sus puntitos
    *Se guarda el instante en el que se solicito
    *Se guarda el instante en el que se envio(publico)
    *Se guarda un flag si fue una solicitud de replanning o que
    */

    /*
   Tambien deberia tener una funcion para guardar el report en un archivito

   */

private:
    std::vector<ReportElement> data;
};

namespace PathPlanners
{
class GlobalPlanner : public ThetaStar
{

    typedef actionlib::SimpleActionClient<upo_actions::ExecutePathAction> ExecutePathClient;
    typedef actionlib::SimpleActionServer<upo_actions::MakePlanAction> MakePlanServer;
    typedef actionlib::SimpleActionClient<upo_actions::RotationInPlaceAction> RotationInPlaceClient;

public:
    //Default constructor
    GlobalPlanner(tf2_ros::Buffer *tfBuffer_, string node_name);
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
    void configParams();
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

    void publishTrajectory();

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
    ros::Subscriber goal_sub, global_costmap_sub;

    //Services servers
    ros::ServiceServer global_replanning_service, reset_global_costmap_service, plan_request_service;
    ros::ServiceClient recovery_rot_srv_client;
    //ThetaStar object
    ThetaStar gbPlanner;

    //tf buffer used to get the base_link position on the map(i.e. tf base_link-map)
    tf2_ros::Buffer *tfBuffer;

    //Old tf1 used by the costmap wrapper
    // tf::TransformListener *tf_list_ptr;
    // costmap_2d::Costmap2DROS *global_costmap_ptr;

    std::unique_ptr<tf::TransformListener> tf_list_ptr;
    std::unique_ptr<costmap_2d::Costmap2DROS> global_costmap_ptr;

    std_msgs::Bool flg_replan_status;

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

    string robot_base_frame, world_frame, node_name;

    //Output variables
    int number_of_points;
    int nbrRotationsExec;
    int seq;
    float pathLength;
    Trajectory trajectory;

    //Control flags
    bool globalGoalReceived,goalRunning;

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

    ros::Time start_time;

}; //class GlobalPlanner

} //namespace PathPlanners

#endif