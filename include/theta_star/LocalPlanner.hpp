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

#include <theta_star/ThetaStar2D.hpp>
#include <theta_star/ThetaStar3D.hpp>

#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/LocalPlannerConfig.h>
#include <theta_star_2d/checkObstacles.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <upo_actions/ExecutePathAction.h>
#include <upo_actions/NavigateAction.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <pcl_ros/point_cloud.h>

namespace PathPlanners
{
class LocalPlanner : public ThetaStar3D
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

    void configTopics();
    //Config thetastar class parameters
    void configTheta();
    void configServices();

    void publishExecutePathFeedback();

    geometry_msgs::TransformStamped getTfMapToRobot();

    void configParams2D();
    void publishTrajMarker2D();
    void buildAndPubTrayectory2D();
    bool calculateLocalGoal2D();
    void calculatePath2D();
    void buildAndPubTrayectory3D();
    void publishTrajMarker3D();
    void configParams3D();
    bool calculateLocalGoal3D();
    void calculatePath3D();

    //Auxiliar functions
    void showTime(string message, struct timeb st, struct timeb ft);
    bool pointInside(geometry_msgs::Vector3 p);
    geometry_msgs::Point makePoint(const geometry_msgs::Vector3 &vec);

    //Add the occupied borders around received costmap
    void inflateCostMap();
    //Set to 0 the positions near the local goal in the border
    void freeLocalGoal();
    void collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg);

    inline double euclideanDistance(double x0, double y0, double x, double y)
    {
        return sqrt(pow(x - x0, 2) + pow(y - y0, 2));
    }
    inline double euclideanDistance(double x0, double y0, double z0, double x, double y, double z)
    {
        return sqrt(pow(x - x0, 2) + pow(y - y0, 2) + pow(z - z0, 2));
    }
    //Variables
    ros::NodeHandlePtr nh;
    ros::ServiceClient costmap_clean_srv;
    ros::Subscriber local_map_sub, pointcloud_sub;
    ros::Publisher visMarkersPublisher, trajPub;
    std::unique_ptr<tf::TransformListener> tf_list;

    //Flags publishers

    bool showConfig;
    bool mapGeometryConfigured;
    bool doPlan;
    //Flow control flags
    //To calculate planning time
    struct timeb startT, finishT;

    //Theta star algorithm parameters
    //Geometric params
    //Algorithm params
    float cost_weight;
    float lof_distance;
    float occ_threshold;
    //Traj params
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

    geometry_msgs::Vector3 local_costmap_center, localGoal, robotPose;
    ThetaStar2D theta2D;
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

    //!
    //Theta star algorithm parameters
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
    bool data_source;

    double arrivedThresh;
    octomap_msgs::OctomapConstPtr map;
    ThetaStar3D theta3D;

    std::shared_ptr<ros::NodeHandle> nh3d;
    bool use3d;
    bool mapReceived;

    trajectory_msgs::MultiDOFJointTrajectoryPoint currentGoal;
    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> goals_vector;
};

} // namespace PathPlanners

#endif