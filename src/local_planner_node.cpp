#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>

#include <theta_star/ThetaStar.hpp>

#include <visualization_msgs/MarkerArray.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define DEBUG

using namespace std;
using namespace PathPlanners;

geometry_msgs::Vector3 local_costmap_center, trans_global, localGoal;
nav_msgs::OccupancyGrid localCostMap;
tf::TransformListener *tf_listener;
trajectory_msgs::MultiDOFJointTrajectory globalTrajectory, localTrajectory;

//Flags for flow control
bool localCostMapReceived = false;
bool globalTrajReceived = false;
bool localGoalReached = true;

struct timeb startT, finishT;

int global_traj_array_length;
int local_traj_array_length;

//Theta star algorithm parameters
double map_resolution = 0.0;
double ws_x_max = 0.0;
double ws_y_max = 0.0;
double ws_z_max = 0.0;
double ws_x_min = 0.0;
double ws_y_min = 0.0;
double map_h_inflaction = 0.0;
double goal_weight = 1.0;
double traj_dxy_max = 0.5;
double traj_vxy_m = 1.0;
double traj_vxy_m_1 = 1.0;
double traj_wyaw_m = 1.0;
double traj_pos_tol = 1.0;
double traj_yaw_tol = 1.0;


//Functions declarations 
geometry_msgs::Vector3 calculateLocalGoal();
geometry_msgs::Transform getRobotPoseInMapFrame();

//Calbacks and publication functions
void localCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &lcp);
void globalTrajCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &traj);
void localGoalReachedCallback(const std_msgs::Bool &data);
void publishTrajMarker(ros::Publisher *traj_marker_pub);
void buildAndPubTrayectory(ThetaStar *theta);
//True if you want to see the params used in the terminal
void configParams(bool showConfig);

//Auxiliar functions
float getYawFromQuat(Quaternion quat);
void printfTrajectory(trajectory_msgs::MultiDOFJointTrajectory traj);
void showTime(string message, struct timeb st, struct timeb ft);

int main(int argc, char **argv)
{

    string node_name = "local_planner_node_2";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    tf_listener = new tf::TransformListener();

    char topicPath[100];

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
    ros::Subscriber global_traj = n.subscribe(topicPath, 1, globalTrajCallback);
    ROS_INFO("Local Planner global trajectory input: %s", topicPath);

    //Trajectory publisher to trajectory tracker node
    sprintf(topicPath, "/trajectory_tracker/local_input_trajectory");
    ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 10);
    ROS_INFO("local planner local trajectory output topic: %s", topicPath);
    // Trajectory solution visualization topic
    ros::Publisher vis_pub_traj = n.advertise<visualization_msgs::MarkerArray>(node_name + "/visualization_marker_trajectory", 10);

    //Load params from parameter server, true if you want to visualize the loaded parameters
    configParams(true);
    // Init Theta*
    ThetaStar theta((char *)node_name.c_str(), (char *)"/world", ws_x_max, ws_y_max, ws_x_min, ws_y_min, map_resolution, map_h_inflaction, goal_weight, &n);

    // Set timeout
    theta.setTimeOut(20);

    // Set resulting trajectroy parameters
    theta.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_vxy_m, traj_vxy_m_1, traj_wyaw_m, traj_yaw_tol);

    ROS_INFO("Waiting for new local goal...");

    while (ros::ok())
    {
        ros::spinOnce();

        //Once a goal is sent to rviz and the global planner compute and publish the trajectory, 
        //the callback is activated, the global trajectory is received and we start doing things

        if (localCostMapReceived && globalTrajReceived)
        {
            ros::spinOnce();

            ROS_INFO("Local costmap received");
            
            theta.getMap(localCostMap);

            ROS_INFO("Local costmap sent to algorithm");

            localGoal = calculateLocalGoal();

            if (theta.setValidInitialPosition(local_costmap_center) && theta.setValidFinalPosition(localGoal))
            {
                // Path calculation
                ROS_INFO("Path calculation...");

                ftime(&startT);
                int number_of_points = theta.computePath();
                ftime(&finishT);

                showTime("Time spend in local path calculation: ", startT, finishT);

                ROS_INFO("Local Trajectories:\n");

                buildAndPubTrayectory(&theta);

                trajectory_pub.publish(localTrajectory);

                printfTrajectory(localTrajectory);
                publishTrajMarker(&vis_pub_traj);
                globalTrajReceived = false;
            }
        }

        usleep(1e5);
    }

    return 0;
}
void buildAndPubTrayectory(ThetaStar *theta)
{
    trajectory_msgs::MultiDOFJointTrajectoryPoint goal;
    geometry_msgs::Transform temp1;

    theta->getTrajectoryYawInAdvance(localTrajectory, getRobotPoseInMapFrame());
    
    local_traj_array_length = localTrajectory.points.size();

    for (int i = 0; i < local_traj_array_length; i++)
    {
        localTrajectory.points[i].transforms[0].translation.x += localCostMap.info.origin.position.x;
        localTrajectory.points[i].transforms[0].translation.y += localCostMap.info.origin.position.y;
    }
    localGoal.x += localCostMap.info.origin.position.x;
    localGoal.y += localCostMap.info.origin.position.y;

    temp1.translation.x = localGoal.x;
    temp1.translation.y = localGoal.y;
    //TODO: PONER LA OPRIENTATION DEL WAYPOINT SIGUIENTE AL LOCAL GOAL
    temp1.rotation.w = 1;
 
    goal.transforms.resize(1,temp1);

    localTrajectory.points.push_back(goal);
    local_traj_array_length = localTrajectory.points.size();

}
geometry_msgs::Vector3 calculateLocalGoal()
{
    //Aux variables, localgoal will be stored in C,
    //A and B will be the points of the trajectory just inside and just outside the local costmap workspace
    geometry_msgs::Vector3 A, B, C, dir;
    //Pose stamped to work with TF
    geometry_msgs::PoseStamped t1, t2;

    bool out = false;

    for (unsigned int i = 0; i < global_traj_array_length; i++)
    {
        //We search the first point of the trajectory outside the local costmap workspace (i)
        if ((fabs(globalTrajectory.points[i].transforms[0].translation.x) > 3) ||
            (fabs(globalTrajectory.points[i].transforms[0].translation.y) > 3))
        {
            //The point i-1 has to be inside the local costmap workspace
            A.x = globalTrajectory.points[i - 1].transforms[0].translation.x;
            A.y = globalTrajectory.points[i - 1].transforms[0].translation.y;
            B.x = globalTrajectory.points[i].transforms[0].translation.x;
            B.y = globalTrajectory.points[i].transforms[0].translation.y;
            dir.x = B.x - A.x;
            dir.y = B.y - A.y;
#ifdef DEBUG
            ROS_INFO("A: [%.2f, %.2f], B: [%.2f, %.2f], dir: [%.2f, %.2f]", A.x, A.y, B.x, B.y, dir.x, dir.y);
#endif
            out = true;
            //Once we found them out, we exit the loop
            break;
        } //If alltrajectory points are inside the local costmap workspace, we take as localGoal( point C) the last point of the global trajectory
        if (i == global_traj_array_length - 1)
        {
            C.x = globalTrajectory.points[global_traj_array_length - 1].transforms[0].translation.x;
            C.y = globalTrajectory.points[global_traj_array_length - 1].transforms[0].translation.y;
#ifdef DEBUG
            ROS_INFO("C:[%.2f, %.2f]", C.x, C.y);
#endif
        }
    }
    //Enter only if there were points from the trajectory outside the local costmap workspace
    if (out)
    {
        C.x = A.x + dir.x / 2;
        C.y = A.y + dir.y / 2;
        while (fabs(C.x) > 3 ||
               fabs(C.y) > 3)
        {
            C.x -= dir.x / 20;
            C.y -= dir.y / 20;
        }
    }
#ifdef DEBUG
    ROS_INFO("Local goal (base_link frame): [%.2f, %.2f]", C.x, C.y);
#endif
    //The C point is in the base link frame so we transform it to map frame and store it in t2
    t1.header.stamp = ros::Time(0);
    t1.header.frame_id = "base_link";
    t1.pose.position.x = C.x;
    t1.pose.position.y = C.y;
    t1.pose.orientation.w = 1;

    tf_listener->transformPose("map", t1, t2);

    //Once t2 is the localGoal in map frame, we transform it to local costmap frame simply translating it
    C.x = t2.pose.position.x - localCostMap.info.origin.position.x;
    C.y = t2.pose.position.y - localCostMap.info.origin.position.y;
    //Refine to get sure the local goal is inside the local costmap worksspaceç
    //Esto es un mecanismo de seguridad cutre, CAMBIARLO
    while (fabs(C.x) > 6)
        C.x -= C.x / 100;

    while (fabs(C.y) > 6)
        C.y -= C.y / 100;
#ifdef DEBUG
    ROS_INFO("Local Goal(LOCAL COSTMAP frame): [%.2f, %.2f]", C.x, C.y);
#endif
    return C;
}

geometry_msgs::Transform getRobotPoseInMapFrame()
{

    geometry_msgs::PoseStamped t1, t2;
    geometry_msgs::Transform ret;

    t1.header.frame_id = "base_link";
    t1.header.stamp = ros::Time(0);
    t1.pose.position.x = 0;
    t1.pose.position.y = 0;
    t1.pose.orientation.w = 1;

    tf_listener->transformPose("map", t1, t2);

    ret.translation.x = t2.pose.position.x + trans_global.x;
    ret.translation.y = t2.pose.position.y + trans_global.y;
    //Pendiente de mandarle la verdadera orientacion del base link
    ret.rotation.w = t2.pose.orientation.w;
    ret.rotation.x = t2.pose.orientation.x;
    ret.rotation.y = t2.pose.orientation.y;
    ret.rotation.z = t2.pose.orientation.z;
    return ret;
}
void globalTrajCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &traj)
{
    if (!globalTrajReceived)
    {
        globalTrajectory = *traj;
        global_traj_array_length = globalTrajectory.points.size();
#ifdef DEBUG
        ROS_INFO("Local planner Node: Global Trajectory received:");
        printfTrajectory(globalTrajectory);
#endif
        //Ahora la ponemos referida a MAP, ya que nos llega tomando el (0,0) como la esquina inferior
        //Seguidamente se cambia a base_link
        globalTrajectory.header.frame_id = "map";

        geometry_msgs::PoseStamped temp, temp1;
        temp.header.frame_id = "map";
        temp.pose.orientation.w = 1;

        for (unsigned int i = 0; i < global_traj_array_length; i++)
        {
            temp.header.stamp = ros::Time(0);
            temp.pose.position.x = globalTrajectory.points[i].transforms[0].translation.x;
            temp.pose.position.y = globalTrajectory.points[i].transforms[0].translation.y;

            tf_listener->transformPose("base_link", temp, temp1);

            globalTrajectory.points[i].transforms[0].translation.x = temp1.pose.position.x;
            globalTrajectory.points[i].transforms[0].translation.y = temp1.pose.position.y;
        }
        globalTrajectory.header.frame_id = "base_link";
#ifdef DEBUG
        printfTrajectory(globalTrajectory);
#endif
        globalTrajReceived = true;
    }
}
void localGoalReachedCallback(const std_msgs::Bool &data)
{
    localGoalReached = data.data;
}
void localCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &lcp)
{
    localCostMap = *lcp;
    localCostMapReceived = true;
}
void publishTrajMarker(ros::Publisher *traj_marker_pub)
{

    visualization_msgs::MarkerArray traj_marker;
    traj_marker.markers.resize(local_traj_array_length);

    for (int i = 0; i < local_traj_array_length; i++)
    {
        traj_marker.markers[i].type = visualization_msgs::Marker::ARROW;
        traj_marker.markers[i].points.clear();
        traj_marker.markers[i].header.frame_id = "map";
        traj_marker.markers[i].header.stamp = ros::Time();
        traj_marker.markers[i].ns = "local_traj";
        traj_marker.markers[i].id = i;
        traj_marker.markers[i].action = visualization_msgs::Marker::ADD;
        traj_marker.markers[i].pose.position.z = 0.5;
        traj_marker.markers[i].scale.x = 0.8;
        traj_marker.markers[i].scale.y = 0.2;
        traj_marker.markers[i].scale.z = 0.2;
        traj_marker.markers[i].color.a = 1.0;
        traj_marker.markers[i].color.r = 1.0;
        traj_marker.markers[i].color.g = 0.0;
        traj_marker.markers[i].color.b = 0.5;
        traj_marker.markers[i].pose.orientation.w = localTrajectory.points[i].transforms[0].rotation.w;
        traj_marker.markers[i].pose.orientation.z = localTrajectory.points[i].transforms[0].rotation.z;
        traj_marker.markers[i].pose.orientation.x = localTrajectory.points[i].transforms[0].rotation.x;
        traj_marker.markers[i].pose.orientation.y = localTrajectory.points[i].transforms[0].rotation.y;
        traj_marker.markers[i].pose.position.x = localTrajectory.points[i].transforms[0].translation.x;
        traj_marker.markers[i].pose.position.y = localTrajectory.points[i].transforms[0].translation.y;
    }
    traj_marker_pub->publish(traj_marker);
}

void configParams(bool showConfig)
{
    ros::param::get("/costmap_2d_local/costmap/width", ws_x_max);
    ros::param::get("/costmap_2d_local/costmap/height", ws_y_max);
    ws_x_min = 0;
    ws_y_min = 0;
    local_costmap_center.x = ws_x_max / 2;
    local_costmap_center.y = ws_x_max / 2;
    ros::param::get("/costmap_2d_local/costmap/resolution", map_resolution);
    ros::param::get("/local_planner/map_h_inflaction", map_h_inflaction);
    ros::param::get("/local_planner/goal_weight", goal_weight);
    ros::param::get("/local_planner/traj_dxy_max", traj_dxy_max);
    ros::param::get("/local_planner/traj_vxy_m", traj_vxy_m);
    ros::param::get("/local_planner/traj_vxy_m_1", traj_vxy_m_1);
    ros::param::get("/local_planner/traj_wyaw_m", traj_wyaw_m);
    ros::param::get("/local_planner/traj_pos_tol", traj_pos_tol);
    ros::param::get("/local_planner/traj_yaw_tol", traj_yaw_tol);

    ros::param::get("/costmap_2d/costmap/origin_x", trans_global.x);
    ros::param::get("/costmap_2d/costmap/origin_y", trans_global.y);
    if (showConfig)
    {
        printf("Local Planner Node Configuration:\n");
        printf("\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] \n", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
        printf("\t Local Costmap Origin: [%.2f, %.2f] \n", local_costmap_center.x, local_costmap_center.y);
        printf("\tGlobal Costmap Origin: [%.2f, %.2f] \n", trans_global.x, trans_global.y);
        printf("\t Map: resol.= [%.2f], inflac.= [%.2f]\n", map_resolution, map_h_inflaction);
        printf("\t Lazy Theta* with optim.: goal_weight = [%.2f]\n", goal_weight);
        printf("\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]\n", traj_dxy_max, traj_pos_tol);
        printf("\t Trajectory Cruising Speed = [%.2f] (%.2f)\n", traj_vxy_m, traj_vxy_m_1);
        printf("\t Trajectory Angular vel: [%.2f], Minimum Dpos: [%.2f]\n\n", traj_wyaw_m, traj_yaw_tol);
    }
}
float getYawFromQuat(Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y;
}
void printfTrajectory(trajectory_msgs::MultiDOFJointTrajectory traj)
{
    printf(PRINTF_YELLOW "Trajectory points: %d\n", (int)traj.points.size());
    for (unsigned int i = 0; i < traj.points.size(); i++)
    {
        double yaw = getYawFromQuat(traj.points[i].transforms[0].rotation);
        printf(PRINTF_BLUE "\t %d: [%.3f, %.3f] m,  [%.2f] grados \n", i, traj.points[i].transforms[0].translation.x, traj.points[i].transforms[0].translation.y, yaw);
    }
    printf(PRINTF_REGULAR);
}
void showTime(string message, struct timeb st, struct timeb ft)
{
    float seconds, milliseconds;

    seconds = ft.time - st.time - 1;
    milliseconds = (1000 - st.millitm) + ft.millitm;
    cout << message << (milliseconds + seconds * 1000) << " ms" << endl;
}