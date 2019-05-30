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

#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/localConfig.h>

//#define DEBUG

using namespace std;
using namespace PathPlanners;

geometry_msgs::Vector3 local_costmap_center, localGoal;

nav_msgs::OccupancyGrid localCostMap, localCostMapInflated;

tf2_ros::Buffer tfBuffer;

trajectory_msgs::MultiDOFJointTrajectory globalTrajectory, localTrajectory;
trajectory_msgs::MultiDOFJointTrajectoryPoint globalGoal;

visualization_msgs::Marker markerTraj;
//Flags for flow control
bool localCostMapReceived = false;
bool globalTrajReceived = false;
bool localGoalReached = true;
bool globalGoalReached = false;

struct timeb startT, finishT;

unsigned int startIter = 0;
int globalTrajArrLen;
int localTrajArrLen;

string robot_base_frame, world_frame;

//Theta star algorithm parameters
double map_resolution = 0.0;
double ws_x_max = 0.0;
double ws_y_max = 0.0;
double ws_x_min = 0.0;
double ws_y_min = 0.0;
double goal_weight = 1.5;
double cost_weight = 0.0;
double lof_distance = 0.0;

double occ_threshold = 80.0;
double traj_dxy_max = 0.5;
double traj_pos_tol = 1.0;
double traj_yaw_tol = 1.0;
double localCostMapInflationX = 1.0;
double localCostMapInflationY = 1.0;
//Functions declarations
geometry_msgs::Vector3 calculateLocalGoal();
geometry_msgs::TransformStamped getTransformFromMapToBaseLink();

void callback(theta_star_2d::localConfig &config, uint32_t level);

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
    //Trajectory publisher to trajectory tracker node
    sprintf(topicPath, "/trajectory_tracker/local_input_trajectory");
    ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 1);
    ROS_INFO("local planner local trajectory output topic: %s", topicPath);
    // Trajectory solution visualization topic
    ros::Publisher vis_pub_traj = n.advertise<visualization_msgs::Marker>(node_name + "/visualization_marker_trajectory", 10);

    ros::Publisher impossible = n.advertise<std_msgs::Bool>("/trajectory_tracker/impossible_to_find",1);
    std_msgs::Bool is_running, occ;
    occ.data = false;
    is_running.data = true;
    ros::Publisher local_run_pub = n.advertise<std_msgs::Bool>("/local_planner_node/running", 10);
    ros::Publisher plan_time = n.advertise<std_msgs::Int32>("/local_planning_time", 1000);
    ros::Publisher occ_goal_pub = n.advertise<std_msgs::Bool>("/trajectory_tracker/local_goal_occupied",1);
    //Dynamic reconfigure
	dynamic_reconfigure::Server<theta_star_2d::localConfig> server;
  	dynamic_reconfigure::Server<theta_star_2d::localConfig>::CallbackType f;

  	f = boost::bind(&callback, _1, _2);
  	server.setCallback(f);

    configParams(false);
    ThetaStar theta((char *)node_name.c_str(), (char *)world_frame.c_str(), ws_x_max, ws_y_max, ws_x_min, ws_y_min, map_resolution, goal_weight,cost_weight,lof_distance,occ_threshold, &n);
    theta.setTimeOut(20);
    theta.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);

    ROS_INFO("Waiting for new local goal...");

    ros::Rate loop_rate(30);
    int number_of_points = 0;
    bool startOk = false;
    float seconds, milliseconds;
    std_msgs::Int32 msg;
    int impossibles = 0;

    while (ros::ok())
    {
        ftime(&startT);
        ros::spinOnce();
        inflateCostMap();
        inf_costmap.publish(localCostMapInflated);
        local_run_pub.publish(is_running);
        configParams(false);
		theta.setDynParams(goal_weight,cost_weight,lof_distance, occ_threshold);

        number_of_points = 0; //Reset variable
        if (globalTrajReceived && localCostMapReceived)
        {
            localCostMapReceived = false;
            if (!theta.setValidInitialPosition(local_costmap_center))
            {
                if (theta.searchInitialPosition2d(0.5))
                {
                    startOk = true;
                }
                else
                {
                    ROS_INFO("Local:Couldn't find a free point near start point");
                }
            }
            else
            {
                startOk = true;
            } //To get sure the next time it calculate a path it has refreshed the local costmap
            if (startOk)
            {
                theta.getMap(&localCostMapInflated);
                localGoal = calculateLocalGoal();
                if (!theta.setValidFinalPosition(localGoal))
                {

                    if (theta.searchFinalPosition2d(0.2))//Estaba a 0.4 antes(en la demo de portugal)
                    {
                        number_of_points = theta.computePath();
                    }
                    else
                    {
                        ROS_INFO("Local: Couldn't find a free point near local goal ");
                        globalTrajReceived = false;
                        occ.data = true;
                        occ_goal_pub.publish(occ);
                    }
                }
                else
                {   if(occ.data){
                        occ.data=false;
                        occ_goal_pub.publish(occ);
                    }
                    number_of_points = theta.computePath();
                }
                if (number_of_points > 0 && !occ.data)
                {
                    buildAndPubTrayectory(&theta, &trajectory_pub);
                    publishTrajMarker(&vis_pub_traj);
                    startOk = false;
                    
                    if(impossible > 0){
                        std_msgs::Bool msg;
                        msg.data  =true;
                        impossible.publish(msg);
                        impossibles = 0;
                    }
                   

                }else{
                    impossibles++;
                    if(impossibles == 3){
                        std_msgs::Bool msg;
                        msg.data  =true;
                        impossible.publish(msg);
                    }
                }
            }
        }

        ftime(&finishT);
        seconds = finishT.time - startT.time - 1;
        milliseconds = (1000 - startT.millitm) + finishT.millitm;
        msg.data = (milliseconds + seconds * 1000);
        plan_time.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}
void callback(theta_star_2d::localConfig &config, uint32_t level) {
  /*ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.double_param, 
            config.size);*/
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

    localTrajectory.header.stamp = ros::Time::now();
    
    traj_publisher->publish(localTrajectory);
    ROS_WARN_THROTTLE(1,"Average dist 2 obstacles: %.2f",theta->getAvDist2Obs());

}
geometry_msgs::Vector3 calculateLocalGoal()
{

    geometry_msgs::Vector3Stamped C;

    trajectory_msgs::MultiDOFJointTrajectory globalTrajBLFrame = globalTrajectory;
    //First we transform the trajectory published by global planner to base_link frame
    geometry_msgs::TransformStamped tr = getTransformFromMapToBaseLink();

    globalTrajBLFrame.header.frame_id = robot_base_frame;

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

        if (fabs(globalTrajBLFrame.points[i].transforms[0].translation.x) > (ws_x_max / 2 - localCostMapInflationX) ||
            fabs(globalTrajBLFrame.points[i].transforms[0].translation.y) > (ws_y_max / 2 - localCostMapInflationY) || i == globalTrajArrLen - 1)
        {
            startIter = i;
            break;
        }
    }
    ROS_INFO_THROTTLE(1,PRINTF_BLUE"[%.2f, %.2f]", C.vector.x,C.vector.y);
    C.vector.x = floor(C.vector.x*10 + 0.5) / 10;
    C.vector.y = floor(C.vector.y*10 + 0.5) / 10;
    if(C.vector.x == 0)
        C.vector.x+=0.05;
    if(C.vector.y == 0)
        C.vector.y+=0.05;
        
    ROS_INFO_THROTTLE(1,PRINTF_BLUE"[%.2f, %.2f]", C.vector.x,C.vector.y);
    return C.vector;
}
geometry_msgs::TransformStamped getTransformFromMapToBaseLink()
{

    geometry_msgs::TransformStamped ret;

    try
    {
        ret = tfBuffer.lookupTransform(world_frame, robot_base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return ret;
}

void publishTrajMarker(ros::Publisher *traj_marker_pub)
{
    markerTraj.points.clear();
    geometry_msgs::Point p;
    for (int i = 0; i < localTrajArrLen; i++)
    {
        p.x = localTrajectory.points[i].transforms[0].translation.x;
        p.y = localTrajectory.points[i].transforms[0].translation.y;
        markerTraj.points.push_back(p);
    }
    traj_marker_pub->publish(markerTraj);
}

void inflateCostMap()
{
    localCostMapInflated.data.resize(0);

    localCostMapInflated.header.frame_id = localCostMap.header.frame_id;
    localCostMapInflated.header.seq = localCostMap.header.seq;
    localCostMapInflated.header.stamp = ros::Time(0);

    localCostMapInflated.info.height = localCostMap.info.height + 2 * localCostMapInflationY / map_resolution;
    localCostMapInflated.info.width = localCostMap.info.width + 2 * localCostMapInflationX / map_resolution;

    localCostMapInflated.info.resolution = map_resolution;
    
    localCostMapInflated.info.origin.position.x = localCostMap.info.origin.position.x - localCostMapInflationX;
    localCostMapInflated.info.origin.position.y = localCostMap.info.origin.position.y - localCostMapInflationY;

    float l = localCostMapInflated.info.width * localCostMapInflationY / map_resolution;

    int iter = 0;
    for (int i = 0; i < l; i++){
        localCostMapInflated.data.push_back(100);   
    }
    for (int i = 0; i < localCostMap.info.height; i++)
    {
        for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
            localCostMapInflated.data.push_back(i==100 || i== localCostMap.info.height-1 ?100:100);

        for (int k = 0; k < localCostMap.info.width; k++)
        {
            localCostMapInflated.data.push_back(localCostMap.data[iter]);
            iter++;
        }
        for (int l = 0; l < localCostMapInflationX / map_resolution; l++)
            localCostMapInflated.data.push_back(i==0 || i== localCostMap.info.height-1 ?100:100);
    }
    for (int i = 0; i < l; i++)
        localCostMapInflated.data.push_back(100);
}
void configParams(bool showConfig)
{
    ros::param::get("/costmap_2d_local/costmap/width", ws_x_max);
    ros::param::get("/costmap_2d_local/costmap/height", ws_y_max);
    ros::param::get("/costmap_2d_local/costmap/resolution", map_resolution);
	ros::param::get("/local_planner_node/goal_weight", goal_weight);
	ros::param::get("/local_planner_node/cost_weight", cost_weight);
	ros::param::get("/local_planner_node/lof_distance", lof_distance);
	ros::param::get("/local_planner_node/occ_threshold", occ_threshold);
    ros::param::get("/local_planner_node/traj_dxy_max", traj_dxy_max);
    ros::param::get("/local_planner_node/traj_pos_tol", traj_pos_tol);
    ros::param::get("/local_planner_node/traj_yaw_tol", traj_yaw_tol);
    ros::param::get("/local_planner_node/world_frame", world_frame);
	ros::param::get("/local_planner_node/robot_base_frame", robot_base_frame);
    ros::param::get("/local_planner_node/local_costmap_infl_x", localCostMapInflationX);
    ros::param::get("/local_planner_node/local_costmap_infl_y", localCostMapInflationY);
    ws_x_max += 2 * localCostMapInflationX;
    ws_y_max += 2 * localCostMapInflationY;

    local_costmap_center.x = ws_x_max / 2;
    local_costmap_center.y = ws_y_max / 2;

    if (showConfig)
    {
        printf("Local Planner Node Configuration:\n");
        printf("\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] \n", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
        printf("\t Local Costmap Origin: [%.2f, %.2f] \n", local_costmap_center.x, local_costmap_center.y);
        printf("\t Map: resol.= [%.2f]\n", map_resolution);
        printf("\t Lazy Theta* with optim.: goal_weight = [%.2f]\n", goal_weight);
        printf("\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]\n", traj_dxy_max, traj_pos_tol);
        printf("\t Local costmap inflation [%.2f, %.2f]\n", localCostMapInflationX,localCostMapInflationY);
    }
    markerTraj.header.frame_id = world_frame;
    markerTraj.header.stamp = ros::Time();
    markerTraj.ns = "local_path";
    markerTraj.id = 12;
    markerTraj.type = RVizMarker::SPHERE_LIST;
    markerTraj.action = RVizMarker::ADD;
    markerTraj.pose.orientation.w = 1.0;
    markerTraj.lifetime = ros::Duration(0.5);
    markerTraj.scale.x = 0.2;
    markerTraj.scale.y = 0.2;
    markerTraj.pose.position.z = 1.0;
    markerTraj.color.a = 1.0;
    markerTraj.color.r = 1.0;
    markerTraj.color.g = 0.0;
    markerTraj.color.b = 0.0;
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
    globalTrajectory.header.frame_id = world_frame;
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
