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
#include <visualization_msgs/MarkerArray.h>

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
    /*
    @brief: goal topic callback 
    */
    void goalCb(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
    /*
    @brief: Service servers callback for replanning, it takes the same goal and launch the planner again with the more recent costmap
    */
    bool replanningSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep);
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

    /*              Class Variables                 */
    ros::NodeHandle nh_;

    visualization_msgs::MarkerArray markerTraj;
    visualization_msgs::Marker marker;
    
    geometry_msgs::PoseStamped goalPoseStamped;
    geometry_msgs::Vector3Stamped goal;
    
    //Publishers and Subscribers
    ros::Publisher trj_pub, vis_trj_pub,replan_status_pub;
    ros::Subscriber goal_sub, global_costmap_sub;
    
    //Services servers
    ros::ServiceServer global_replanning_service, reset_global_costmap_service;
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
    Trajectory trajectory;
    
    //Control flags
    bool globalGoalReceived;
    
    //These two flags can be configured as parameters
    bool showConfig, debug;

    int countImpossible = 0;

}; //class GlobalPlanner

} //namespace PathPlanners

#endif