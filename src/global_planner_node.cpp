#include <iostream>
#include <cstdlib>
#include <string>
#include <ctime>
#include <sys/timeb.h>
#include <math.h>
#include <ros/ros.h>
#include <fstream>

#include <theta_star/ThetaStar.hpp>

#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/globalConfig.h>

//#define DEBUG

using namespace std;
using namespace PathPlanners;

visualization_msgs::Marker markerTraj;
geometry_msgs::PoseStamped goalPoseStamped;
geometry_msgs::Vector3Stamped goal;
nav_msgs::OccupancyGrid globalCostMap;

nav_msgs::OccupancyGrid::ConstPtr gCmPtr;

tf2_ros::Buffer tfBuffer;

bool globalGoalReceived = false;
bool gCmPassed = false;
bool gCmReceived = false;
bool globalGoalSet = false;
bool initialPoseSet = false;
bool globalTrajSent = false;

struct timeb startT, finishT;

int number_of_points;

string robot_base_frame, world_frame;

float map_resolution;
float ws_x_max;
float ws_y_max;
float ws_x_min;
float ws_y_min;
float cost_weight;
float goal_weight ;
float occ_threshold;
float lof_distance;
float traj_dxy_max;
float traj_pos_tol;
float traj_yaw_tol;

void callback(theta_star_2d::globalConfig &config, uint32_t level);
void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &fp);
void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);

bool setGoal(geometry_msgs::Vector3Stamped, ThetaStar *th);
bool setStart(ThetaStar *th);

geometry_msgs::TransformStamped getTransformFromBaseLinkToMap();

void getAndPublishTrajMarkArray(ros::Publisher *traj_pub, ros::Publisher *vistraj_pub, ThetaStar *th);
void showTime(string message, struct timeb st, struct timeb ft);
void configParams(bool showConfig, ros::NodeHandle *n);

int main(int argc, char **argv)
{

	string node_name = "global_planner_node";
	ros::init(argc, argv, node_name);
	ros::NodeHandle n;

	tf2_ros::TransformListener tfListener(tfBuffer);

	char topicPath[100];

	ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1, rvizGoalCallback);
	// Global costmap topic subscriber
	sprintf(topicPath, "/costmap_2d/costmap/costmap");
	ros::Subscriber global_sub_map = n.subscribe(topicPath, 1, globalCostMapCallback);
	ROS_INFO("Theta Star: Global costmap input topic: %s", topicPath);

	// Trajectory list topic publisher to trajectory_tracker_node
	sprintf(topicPath, "trajectory_tracker/input_trajectory");
	ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 10);
	ROS_INFO("Theta Star: trajectory output topic: %s", topicPath);

	// Trajectory solution visualization topic
    ros::Publisher vis_pub_traj = n.advertise<visualization_msgs::Marker>(node_name + "/visualization_marker_trajectory", 10);


	//Dynamic reconfigure
	dynamic_reconfigure::Server<theta_star_2d::globalConfig> server;
  	dynamic_reconfigure::Server<theta_star_2d::globalConfig>::CallbackType f;
  	f = boost::bind(&callback, _1, _2);
  	server.setCallback(f);
	
	configParams(true, &n);

	ThetaStar theta((char *)node_name.c_str(), (char *)world_frame.c_str(), ws_x_max, ws_y_max, ws_x_min, ws_y_min, map_resolution, goal_weight, cost_weight,lof_distance,occ_threshold, &n);
	ROS_WARN("\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] , step %.2f\n", ws_x_min, ws_x_max, ws_y_min, ws_y_max,map_resolution);
	theta.setTimeOut(20);
	theta.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);
	//ROS_ERROR("4");
	ROS_INFO("Waiting for new global goal...");

	ros::Rate loop_rate(5);
	
	while (ros::ok())
	{
		
		ros::spinOnce();
		//configParams(false,&n);	
		
		theta.setDynParams(goal_weight,cost_weight,lof_distance, occ_threshold);

		if (gCmReceived)
		{
			//ROS_INFO("Sending");
			theta.getMap(&globalCostMap);
			gCmReceived = false;
			ROS_WARN("Global costmap passed to algorithm");
		}

		if (globalGoalReceived)
		{	
			if (setGoal(goal, &theta) && setStart(&theta))
			{
				// Path calculation
				ROS_INFO("Path calculation...");

				number_of_points = theta.computePath();

				if (number_of_points > 0)
				{
					ROS_INFO("Number of points: %d", number_of_points);
					getAndPublishTrajMarkArray(&trajectory_pub, &vis_pub_traj, &theta);
				}
				globalGoalReceived = false;
			}
			else
			{
				ROS_ERROR("Global goal or start point occupied ");
				globalGoalReceived = false;
			}
		}
		loop_rate.sleep();
	}

	return 0;
}
void callback(theta_star_2d::globalConfig &config, uint32_t level) {

}
//Since the local costmap y constant over time, it's not neccesary to refresh it every ros::spinOnce
void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &fp)
{
	gCmPtr = fp;
	globalCostMap = *fp;
	gCmReceived = true;
}
//TODO: delete robotPoseReceived flag change from this callback
//Right now is here to refresh the current robot pose every time a goal is received, but this flag has to be changed inside the main loop
void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goalMsg)
{
	goalPoseStamped.pose = goalMsg->pose;
	goal.vector.x = goalMsg->pose.position.x;
	goal.vector.y = goalMsg->pose.position.y;
	goal.header = goalMsg->header;
	globalGoalReceived = true;
	gCmPassed = false;
}
//This function obtains the transform between map and base_link to send it to the trayectory calculation function from ThetaStar class
geometry_msgs::TransformStamped getTransformFromBaseLinkToMap()
{
	geometry_msgs::TransformStamped ret;

	try
	{
		ret = tfBuffer.lookupTransform(world_frame, robot_base_frame, ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("Global Planner: tf exception: %s", ex.what());
	}

	return ret;
}
bool setGoal(geometry_msgs::Vector3Stamped goal, ThetaStar *th)
{
	if (th->setValidFinalPosition(goal.vector))
	{
		return true;//globalGoalSet = true;
	}
	else
	{
		ROS_ERROR("Global Planner: Failed to set final global position");
		return false;
	}
}
bool setStart(ThetaStar *th)
{

	geometry_msgs::Vector3Stamped start;
	start.vector.x = getTransformFromBaseLinkToMap().transform.translation.x;
	start.vector.y = getTransformFromBaseLinkToMap().transform.translation.y;

	if (th->setValidInitialPosition(start.vector))
	{
		return true;//	initialPoseSet = true;
	}
	else
	{
		ROS_ERROR("Global Planner: Failed to set initial global position");
		return false;
	}
}

void getAndPublishTrajMarkArray(ros::Publisher *traj_pub, ros::Publisher *vistraj_pub, ThetaStar *th)
{

	Trajectory trajectory;

	trajectory.joint_names.push_back(world_frame);
	trajectory.header.stamp = ros::Time::now();
	trajectory.header.frame_id = world_frame;

	trajectory.points.clear();
	trajectory.header.stamp = ros::Time::now();
	ROS_INFO("Trajectory calculation...");

	geometry_msgs::TransformStamped transform_robot_pose = getTransformFromBaseLinkToMap();

	if (number_of_points > 1)
	{
		th->getTrajectoryYawInAdvance(trajectory, transform_robot_pose.transform);
	}
	else
	{
		th->getTrajectoryYawFixed(trajectory, th->getYawFromQuat(transform_robot_pose.transform.rotation));
		trajectory_msgs::MultiDOFJointTrajectoryPoint pos;
		pos.transforms.push_back(transform_robot_pose.transform);
		trajectory.points.push_back(pos);
	}
	// Send the trajectory
	// Trajectory solution visualization marker

	trajectory_msgs::MultiDOFJointTrajectoryPoint goal_multidof;
	geometry_msgs::Transform transform_goal;

	transform_goal.rotation = goalPoseStamped.pose.orientation;
	transform_goal.translation.x = goalPoseStamped.pose.position.x;
	transform_goal.translation.y = goalPoseStamped.pose.position.y;

	goal_multidof.transforms.resize(1, transform_goal);

	trajectory.points.push_back(goal_multidof);

	traj_pub->publish(trajectory);
	
	markerTraj.points.clear();
    geometry_msgs::Point p;
    for (int i = 0; i < trajectory.points.size(); i++)
    {
        p.x = trajectory.points[i].transforms[0].translation.x;
        p.y = trajectory.points[i].transforms[0].translation.y;
        markerTraj.points.push_back(p);
    }
	vistraj_pub->publish(markerTraj);
	globalTrajSent = true;
}
void showTime(string message, struct timeb st, struct timeb ft)
{

	float seconds, milliseconds;

	seconds = ft.time - st.time - 1;
	milliseconds = (1000 - st.millitm) + ft.millitm;
	cout << message << (milliseconds + seconds * 1000) << " ms" << endl;
}
void configParams(bool showConfig, ros::NodeHandle *n)
{
		
	n->param("/global_planner_node/ws_x_max",       ws_x_max,(float)6);
	n->param("/global_planner_node/ws_y_max",       ws_y_max, (float)6);
	n->param("/global_planner_node/map_resolution", map_resolution, (float)0.05);
	n->param("/global_planner_node/goal_weight",    goal_weight, (float)1.5);
	n->param("/global_planner_node/cost_weight",    cost_weight, (float)0.2);
	n->param("/global_planner_node/lof_distance",   lof_distance, (float)0.2);
	n->param("/global_planner_node/occ_threshold",  occ_threshold, (float)99);
	n->param("/global_planner_node/traj_dxy_max",   traj_dxy_max, (float)10);
	n->param("/global_planner_node/traj_pos_tol",   traj_pos_tol, (float)10);
	n->param("/global_planner_node/traj_yaw_tol",   traj_yaw_tol, (float)0.2);
	n->param("/global_planner_node/world_frame",    world_frame, (string)"/map");
	n->param("/global_planner_node/robot_base_frame", robot_base_frame, (string)"/base_link");
	
	ws_x_min = 0;
	ws_y_min = 0;
	

	if (showConfig)
	{
		printf("Global Planner Node Configuration:\n");
		printf("\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] \n", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
		printf("\t Map: resol.= [%.2f]\n", map_resolution);
		printf("\t Lazy Theta* with optim.: goal_weight = [%.2f]\n", goal_weight);
		printf("\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]\n", traj_dxy_max, traj_pos_tol);
	}

	markerTraj.header.frame_id = world_frame;
    markerTraj.header.stamp = ros::Time();
    markerTraj.ns = "global_path";
    markerTraj.id = 12221;
    markerTraj.type = RVizMarker::CUBE_LIST;
    markerTraj.action = RVizMarker::ADD;
    markerTraj.pose.orientation.w = 1.0;
    markerTraj.lifetime = ros::Duration(200);
    markerTraj.scale.x = 0.1;
    markerTraj.scale.y = 0.1;
    markerTraj.pose.position.z = 0.2;
    markerTraj.color.a = 1.0;
    markerTraj.color.r = 0.0;
    markerTraj.color.g = 1.0;
    markerTraj.color.b = 0.0;
}