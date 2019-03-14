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

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>

#include <theta_star/ThetaStar.hpp>

#include <visualization_msgs/MarkerArray.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

//#define DEBUG

using namespace std;
using namespace PathPlanners;

geometry_msgs::Vector3 local_costmap_center, localGoal;

nav_msgs::OccupancyGrid localCostMap, localCostMapInflated;

tf2_ros::Buffer tfBuffer;

trajectory_msgs::MultiDOFJointTrajectory globalTrajectory, localTrajectory;
trajectory_msgs::MultiDOFJointTrajectoryPoint globalGoal;

//Flags for flow control
bool localCostMapReceived = false;
bool globalTrajReceived = false;
bool localGoalReached = true;
bool globalGoalReached = false;

struct timeb startT, finishT;

unsigned int startIter = 0;
int globalTrajArrLen;
int localTrajArrLen;

//Theta star algorithm parameters 
double map_resolution = 0.0;
double ws_x_max = 0.0;
double ws_y_max = 0.0;
double ws_x_min = 0.0;
double ws_y_min = 0.0;
double goal_weight = 1.0;
double traj_dxy_max = 0.5;
double traj_pos_tol = 1.0;
double traj_yaw_tol = 1.0;
double localCostMapInflation = 1.0;

//Functions declarations
geometry_msgs::Vector3 calculateLocalGoal();
geometry_msgs::TransformStamped getTransformFromMapToBaseLink();

void publishTrajMarker(ros::Publisher *traj_marker_pub);
void buildAndPubTrayectory(ThetaStar *theta, ros::Publisher *traj_publisher);

//True if you want to see the params used in the terminal
void configParams(bool showConfig);

//Auxiliar functions
void showTime(string message, struct timeb st, struct timeb ft);
void inflateCostMap();

//Calbacks and publication functions
void localCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &lcp);
void globalTrajCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &traj);
void localGoalReachedCallback(const std_msgs::Bool &data);

int main(int argc, char **argv)
{

    string node_name = "local_planner_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    tf2_ros::TransformListener tfListener(tfBuffer);
    char topicPath[100];

    localGoal.x = 0;
    localGoal.y = 0;

    //TEST: Inflated local costmap topic publisher
    ros::Publisher inf_costmap = n.advertise<nav_msgs::OccupancyGrid>("/local_costmap_inflated", 1);
    // Local costmap topic susbcriber
    sprintf(topicPath, "/costmap_2d_local/costmap/costmap");
    ros::Subscriber local_sub_map = n.subscribe(topicPath, 0, localCostMapCallback);
    ROS_INFO("Theta Star: Local costmap input topic: %s", topicPath);
    //Trajectory tracker local goal reached flag
    sprintf(topicPath, "/trajectory_tracker/local_goal_reached");
    ros::Subscriber local_goal_reached = n.subscribe(topicPath, 1, localGoalReachedCallback);
    ROS_INFO("Trajectory tracker local goal reached flag topic: %s", topicPath);

    //Global trajectory subscriber
    sprintf(topicPath, "/trajectory_tracker/input_trajectory");
    ros::Subscriber global_traj = n.subscribe(topicPath, 10, globalTrajCallback);
    ROS_INFO("Local Planner global trajectory input: %s", topicPath);
    //Map server subscriber to get global translation infor
    //Trajectory publisher to trajectory tracker node
    sprintf(topicPath, "/trajectory_tracker/local_input_trajectory");
    ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 10);
    ROS_INFO("local planner local trajectory output topic: %s", topicPath);
    // Trajectory solution visualization topic
    ros::Publisher vis_pub_traj = n.advertise<visualization_msgs::MarkerArray>(node_name + "/visualization_marker_trajectory", 10);

    std_msgs::Bool is_running;
    is_running.data = true;
    ros::Publisher local_run_pub = n.advertise<std_msgs::Bool>("/local_planner_node/running", 10);
    ros::Publisher plan_time = n.advertise<std_msgs::Int32>("/local_planning_time", 1000);
    
    configParams(false);
    ThetaStar theta((char *)node_name.c_str(), (char *)"/world", ws_x_max, ws_y_max, ws_x_min, ws_y_min, map_resolution, goal_weight, &n);
    theta.setTimeOut(20);
    theta.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);

    ROS_INFO("Waiting for new local goal...");

    ros::Rate loop_rate(20);
    int number_of_points = 0;
    bool startOk = false;
    float seconds, milliseconds;
    std_msgs::Int32 msg;

    while (ros::ok())
    {

        ros::spinOnce();
        inflateCostMap();
        inf_costmap.publish(localCostMapInflated);
        local_run_pub.publish(is_running);

        if (globalTrajReceived && localCostMapReceived)
        {

            localCostMapReceived = false;
            if (!theta.setValidInitialPosition(local_costmap_center))
            {
                if (theta.searchInitialPosition2d(0.3))
                {
                    startOk = true;
                }
                else
                {
                    ROS_INFO("Couldn't find a free point near start point");
                }
            }
            else
            {
                startOk = true;
            } //To get sure the next time it calculate a path it has refreshed the local costmap
            if (startOk)
            {
                theta.getMap(localCostMapInflated);
                localGoal = calculateLocalGoal();
                if (!theta.setValidFinalPosition(localGoal))
                {

                    if (theta.searchFinalPosition2d(0.3))
                    {
                        ftime(&startT);
                        number_of_points = theta.computePath();
                        ftime(&finishT);
                        seconds = finishT.time - startT.time - 1;
                        milliseconds = (1000 - startT.millitm) + finishT.millitm;
                        msg.data = (milliseconds + seconds * 1000);
                        plan_time.publish(msg);
                    }
                    else
                    {
                        ROS_INFO("Couldn't find a free point near local goal ");
                        globalTrajReceived = false;
                    }
                }
                else
                {
                    ftime(&startT);
                    number_of_points = theta.computePath();
                    ftime(&finishT);
                    seconds = finishT.time - startT.time - 1;
                    milliseconds = (1000 - startT.millitm) + finishT.millitm;
                    msg.data = (milliseconds + seconds * 1000);
                    plan_time.publish(msg);
                }
                if (number_of_points > 0)
                {
                    buildAndPubTrayectory(&theta, &trajectory_pub);
                    publishTrajMarker(&vis_pub_traj);
                    startOk = false;
                }
            }
        }
        loop_rate.sleep();
    }

    return 0;
}
void buildAndPubTrayectory(ThetaStar *theta, ros::Publisher *traj_publisher)
{
    trajectory_msgs::MultiDOFJointTrajectoryPoint goal_temp;
    geometry_msgs::Transform temp1;
    //La trayectoria se obtiene en el frame del local costmap, que tiene la misma orientacion que el map pero esta centrado en el base_link
    localTrajectory.points.clear();
    theta->getTrajectoryYawInAdvance(localTrajectory, getTransformFromMapToBaseLink().transform);

    localTrajArrLen = localTrajectory.points.size();

    for (int i = 0; i < localTrajArrLen; i++)
    {
        localTrajectory.points[i].transforms[0].translation.x += localCostMapInflated.info.origin.position.x;
        localTrajectory.points[i].transforms[0].translation.y += localCostMapInflated.info.origin.position.y;
    }
    localGoal.x += localCostMapInflated.info.origin.position.x;
    localGoal.y += localCostMapInflated.info.origin.position.y;

    temp1.translation.x = localGoal.x;
    temp1.translation.y = localGoal.y;
    //TODO: PONER LA OPRIENTATION DEL WAYPOINT SIGUIENTE AL LOCAL GOAL
    temp1.rotation.w = 1;

    goal_temp.transforms.resize(1, temp1);

    localTrajectory.points.push_back(goal_temp);
    localTrajArrLen = localTrajectory.points.size();
  
    traj_publisher->publish(localTrajectory);
}
geometry_msgs::Vector3 calculateLocalGoal()
{

    geometry_msgs::Vector3Stamped C;

    //First we transform the trajectory published by global planner to base_link frame
    geometry_msgs::TransformStamped tr = getTransformFromMapToBaseLink();

    trajectory_msgs::MultiDOFJointTrajectory globalTrajBLFrame = globalTrajectory;
    globalTrajBLFrame.header.frame_id = "base_link";
    for (int i = 0; i < globalTrajArrLen; i++)
    {
        globalTrajBLFrame.points[i].transforms[0].translation.x -= tr.transform.translation.x;
        globalTrajBLFrame.points[i].transforms[0].translation.y -= tr.transform.translation.y;
    }
  
    //Ya esta referida al base_link. Ahora la recorro desde i=1(porque i=0 es siempre la pos del base_link que alpasarla al sistema base_link sera (0,0))
    for (int i = startIter; i < globalTrajArrLen; i++)
    {
        C.vector.x = globalTrajBLFrame.points[i].transforms[0].translation.x + (ws_x_max) / 2;
        C.vector.y = globalTrajBLFrame.points[i].transforms[0].translation.y + (ws_y_max) / 2;

        if (fabs(globalTrajBLFrame.points[i].transforms[0].translation.x) > (ws_x_max / 2 - localCostMapInflation) ||
            fabs(globalTrajBLFrame.points[i].transforms[0].translation.y) > (ws_y_max / 2 - localCostMapInflation) || i == globalTrajArrLen - 1)
        {
            startIter = i;
            break;
        }
    }
    return C.vector;
}
geometry_msgs::TransformStamped getTransformFromMapToBaseLink()
{

    geometry_msgs::TransformStamped ret;

    try
    {
        ret = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return ret;
}

void publishTrajMarker(ros::Publisher *traj_marker_pub)
{

    visualization_msgs::MarkerArray traj_marker;
    traj_marker.markers.resize(localTrajArrLen);

    for (int i = 0; i < localTrajArrLen; i++)
    {
        traj_marker.markers[i].type = visualization_msgs::Marker::CUBE;
        traj_marker.markers[i].points.clear();
        traj_marker.markers[i].header.frame_id = "map";
        traj_marker.markers[i].header.stamp = ros::Time();
        traj_marker.markers[i].ns = "local_traj";
        traj_marker.markers[i].id = i;
        traj_marker.markers[i].action = visualization_msgs::Marker::ADD;
        traj_marker.markers[i].pose.position.z = 0.5;
        traj_marker.markers[i].scale.x = 0.3;
        traj_marker.markers[i].scale.y = 0.3;
        traj_marker.markers[i].scale.z = 0.3;
        traj_marker.markers[i].color.a = 1.0;
        traj_marker.markers[i].color.r = 1.0;
        traj_marker.markers[i].color.g = 0.0;
        traj_marker.markers[i].color.b = 0.5;
        traj_marker.markers[i].lifetime = ros::Duration(1);
        traj_marker.markers[i].pose.orientation.w = localTrajectory.points[i].transforms[0].rotation.w;
        traj_marker.markers[i].pose.orientation.z = localTrajectory.points[i].transforms[0].rotation.z;
        traj_marker.markers[i].pose.orientation.x = localTrajectory.points[i].transforms[0].rotation.x;
        traj_marker.markers[i].pose.orientation.y = localTrajectory.points[i].transforms[0].rotation.y;
        traj_marker.markers[i].pose.position.x = localTrajectory.points[i].transforms[0].translation.x;
        traj_marker.markers[i].pose.position.y = localTrajectory.points[i].transforms[0].translation.y;
    }
    traj_marker_pub->publish(traj_marker);
}

void inflateCostMap()
{
    localCostMapInflated.data.resize(0);

    localCostMapInflated.header.frame_id = localCostMap.header.frame_id;
    localCostMapInflated.header.seq = localCostMap.header.seq;
    localCostMapInflated.header.stamp = ros::Time(0);

    localCostMapInflated.info.height = localCostMap.info.height + 2 * localCostMapInflation / map_resolution;
    localCostMapInflated.info.width = localCostMap.info.width + 2 * localCostMapInflation / map_resolution;
    localCostMapInflated.info.resolution = map_resolution;
    localCostMapInflated.info.origin.position.x = localCostMap.info.origin.position.x - localCostMapInflation;
    localCostMapInflated.info.origin.position.y = localCostMap.info.origin.position.y - localCostMapInflation;

    float h = (2 * localCostMapInflation) / map_resolution + localCostMap.info.width;
    float l = h * localCostMapInflation / map_resolution;

    int iter = 0;

    for (int i = 0; i < l; i++)
        localCostMapInflated.data.push_back(0);

    for (int i = 0; i < localCostMap.info.height; i++)
    {
        for (int j = 0; j < localCostMapInflation / map_resolution; j++)
            localCostMapInflated.data.push_back(0);

        for (int k = 0; k < localCostMap.info.width; k++)
        {
            localCostMapInflated.data.push_back(localCostMap.data[iter]);
            iter++;
        }
        for (int l = 0; l < localCostMapInflation / map_resolution; l++)
            localCostMapInflated.data.push_back(0);
    }
    for (int i = 0; i < l; i++)
        localCostMapInflated.data.push_back(0);
}
void configParams(bool showConfig)
{
    ros::param::get("/costmap_2d_local/costmap/width", ws_x_max);
    ros::param::get("/costmap_2d_local/costmap/height", ws_y_max);
    ros::param::get("/costmap_2d_local/costmap/resolution", map_resolution);
    ros::param::get("/local_planner/goal_weight", goal_weight);
    ros::param::get("/local_planner/traj_dxy_max", traj_dxy_max);
    ros::param::get("/local_planner/traj_pos_tol", traj_pos_tol);
    ros::param::get("/local_planner/traj_yaw_tol", traj_yaw_tol);
    ros::param::get("/local_planner/local_costmap_infl", localCostMapInflation);
    ws_x_max += 2 * localCostMapInflation;
    ws_y_max += 2 * localCostMapInflation;

    local_costmap_center.x = ws_x_max / 2;
    local_costmap_center.y = ws_x_max / 2;

    if (showConfig)
    {
        printf("Local Planner Node Configuration:\n");
        printf("\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] \n", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
        printf("\t Local Costmap Origin: [%.2f, %.2f] \n", local_costmap_center.x, local_costmap_center.y);
        printf("\t Map: resol.= [%.2f]\n", map_resolution);
        printf("\t Lazy Theta* with optim.: goal_weight = [%.2f]\n", goal_weight);
        printf("\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]\n", traj_dxy_max, traj_pos_tol);
        printf("\t Local costmap inflation %.2f\n", localCostMapInflation);
    }
}
void showTime(string message, struct timeb st, struct timeb ft)
{
    float seconds, milliseconds;

    seconds = ft.time - st.time - 1;

    milliseconds = (1000 - st.millitm) + ft.millitm;
    cout << message << (milliseconds + seconds * 1000) << " ms" << endl;
}
void globalTrajCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &traj)
{

    globalTrajectory = *traj;
    globalTrajArrLen = globalTrajectory.points.size();
    startIter = 1;
    globalTrajectory.header.frame_id = "map";
    globalTrajReceived = true;
    globalGoalReached = false;
}
void localGoalReachedCallback(const std_msgs::Bool &data)
{
    globalGoalReached = true;
    globalTrajReceived = !data.data;
}
void localCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &lcp)
{
    localCostMap = *lcp;
    localCostMapReceived = true;
}
