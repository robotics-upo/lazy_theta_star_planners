#include <iostream>
#include <cstdlib>
#include <string>
#include <ctime>
#include <sys/timeb.h>
#include <math.h>

#include <ros/ros.h>

#include <theta_star/ThetaStar.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace PathPlanners;

geometry_msgs::PoseStamped msgGoalPoseStamped, goalPoseStamped;
geometry_msgs::Transform odom_pose;
geometry_msgs::Vector3 origin, goal, init;
nav_msgs::OccupancyGrid globalCostMap, localCostMap;
tf::TransformListener *tf_listener;

bool localCostMapReceived = false;
bool globalGoalReceived = false;
bool odomReceived = false;
bool globalCostMapSentToAlgorithm = false;
bool globalCostMapReceived = false;
bool globalGoalSet = false;
bool amclPoseReceived = false;
bool globalTrajSent = false;
struct timeb startT, finishT;

int traj_array_length;

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

void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &fp);
void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);

//The inputs x_ etc are pixels(more confortable sometimes)
void setGoal(float x_, float y_, float z_, ThetaStar *th);
//The inputs x_ etc are pixels(more confortable sometimes)
void setIni(float x_, float y_, float z_, ThetaStar *th);

void getAndPublishTrajMarkArray(ros::Publisher *traj_pub, ros::Publisher *vistraj_pub, ThetaStar *th);
void showTime(string message, struct timeb st, struct timeb ft);
void configParams(bool showConfig);

int main(int argc, char **argv)
{

	string node_name = "global_planner_node";
	ros::init(argc, argv, node_name);
	ros::NodeHandle n;
	tf_listener = new tf::TransformListener();

	char topicPath[100];

	ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1000, rvizGoalCallback);
	ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odomCallback);
	// Global costmap topic subscriber
	sprintf(topicPath, "/costmap_2d/costmap/costmap");
	ros::Subscriber global_sub_map = n.subscribe(topicPath, 0, globalCostMapCallback);
	ROS_INFO("Theta Star: Global costmap input topic: %s", topicPath);

	// Trajectory list topic publisher to trajectory_tracker_node
	sprintf(topicPath, "trajectory_tracker/input_trajectory");
	ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 10);
	ROS_INFO("Theta Star: trajectory output topic: %s", topicPath);

	// Trajectory solution visualization topic
	ros::Publisher vis_pub_traj = n.advertise<visualization_msgs::MarkerArray>(node_name + "/visualization_marker_trajectory", 10);

	//ros::ServiceServer service = n.advertiseService("getGlobalTrajFlag", getFlagService);

	configParams(false);
	// Init Theta*
	ThetaStar theta((char *)node_name.c_str(), (char *)"/world", ws_x_max, ws_y_max, ws_x_min, ws_y_min, map_resolution, map_h_inflaction, goal_weight, &n);

	// Set timeout
	theta.setTimeOut(20);

	// Set resulting trajectroy parameters
	theta.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_vxy_m, traj_vxy_m_1, traj_wyaw_m, traj_yaw_tol);

	ROS_INFO("Waiting for new global goal...");

	while (ros::ok())
	{
		// Waiting for new goal and reading odometry
		ros::spinOnce();

		if (globalCostMapReceived && !globalCostMapSentToAlgorithm)
		{
			ROS_INFO("Map received");
			theta.getMap(globalCostMap);
			globalCostMapSentToAlgorithm = true;
		}

		if (globalGoalReceived && globalCostMapSentToAlgorithm && odomReceived)
		{
			setGoal(goalPoseStamped.pose.position.x, goalPoseStamped.pose.position.y, 0, &theta);
			setIni(odom_pose.translation.x, odom_pose.translation.y, 0, &theta);

			// Path calculation
			ROS_INFO("Path calculation...");

			ftime(&startT);
			int number_of_points = theta.computePath();
			ftime(&finishT);

			showTime("Time spend in path calculation: ", startT, finishT);

			ROS_INFO("Number of points: %d", number_of_points);

			getAndPublishTrajMarkArray(&trajectory_pub, &vis_pub_traj, &theta);

			globalGoalReceived = false;
			
			//ROS_INFO("Waiting for new global goal...");
		}

		usleep(1e5);
	}

	return 0;
}
void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &fp)
{
	if (!globalCostMapReceived)
	{
		globalCostMap = *fp;
		globalCostMapReceived = true;
	}
}
void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goalMsg)
{
	goalPoseStamped.pose = goalMsg->pose;
	globalGoalReceived = true;
	odomReceived = false;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
	if (!odomReceived)
	{
		geometry_msgs::PoseStamped myOdomPoseStamped, myMapPoseStamped;
		myOdomPoseStamped.header.frame_id = "odom";
		myOdomPoseStamped.header.stamp = ros::Time(0);
		myMapPoseStamped.header.frame_id = "map";
		myMapPoseStamped.header.stamp = ros::Time(0);
		myOdomPoseStamped.pose = odomMsg->pose.pose;
		try
		{
			tf_listener->transformPose("map", myOdomPoseStamped, myMapPoseStamped);
			odomReceived = true;
		}
		catch (const tf2::LookupException &e)
		{
			ROS_INFO("exception in tf listenner");
			odomReceived = false;
		}

		odom_pose.translation.x = myMapPoseStamped.pose.position.x;
		odom_pose.translation.y = myMapPoseStamped.pose.position.y;
		odom_pose.translation.z = 0;
		odom_pose.rotation.w = 1;
		odom_pose.rotation.x = 0;
		odom_pose.rotation.y = 0;
		odom_pose.rotation.z = 0;
		odomReceived = true;
	}
}
//The inputs x_ etc are pixels(more confortable sometimes)
void setGoal(float x_, float y_, float z_, ThetaStar *th)
{
	geometry_msgs::Vector3 pos;
	pos.x = x_ - origin.x;
	pos.y = y_ - origin.y;
	pos.z = z_;
	if (th->setValidFinalPosition(pos))
	{
		goal.x = x_ - origin.x;
		goal.y = y_ - origin.y;
		goal.z = z_;
		globalGoalSet = true;
	}
	else
	{
		ROS_WARN("Failed to set final position");
	}
}
//The inputs x_ etc are pixels(more confortable sometimes)
void setIni(float x_, float y_, float z_, ThetaStar *th)
{
	geometry_msgs::Vector3 pos;
	pos.x = x_ - origin.x;
	pos.y = y_ - origin.y;
	pos.z = z_;
	if (th->setValidInitialPosition(pos))
	{
		init.x = x_ - origin.x;
		init.y = y_ - origin.y;
		init.z = z_;
	}
	else
	{
		ROS_WARN("Failed to set initial position");
	}
}
void getAndPublishTrajMarkArray(ros::Publisher *traj_pub, ros::Publisher *vistraj_pub, ThetaStar *th)
{

	Trajectory trajectory;
	visualization_msgs::MarkerArray traj_marker;
	trajectory.joint_names.push_back("base_link");
	trajectory.header.stamp = ros::Time::now();
	trajectory.header.frame_id = "base_link";

	trajectory.points.clear();
	trajectory.header.stamp = ros::Time::now();
	ROS_INFO("Trajectory calculation...");

	th->getTrajectoryYawInAdvance(trajectory, odom_pose);

	// Send the trajectory
	// Trajectory solution visualization marker

	for(int i = 0; i < trajectory.points.size(); i++  ){
		trajectory.points[i].transforms[0].translation.x += origin.x;
		trajectory.points[i].transforms[0].translation.y += origin.y;
	}
	traj_pub->publish(trajectory);
	traj_array_length = trajectory.points.size() + 1;
	traj_marker.markers.resize(traj_array_length);

	for (int i = 0; i <= trajectory.points.size(); i++)
	{
		traj_marker.markers[i].type = visualization_msgs::Marker::ARROW;
		traj_marker.markers[i].points.clear();
		traj_marker.markers[i].header.frame_id = "map";
		traj_marker.markers[i].header.stamp = ros::Time();
		traj_marker.markers[i].ns = "theta_star";
		traj_marker.markers[i].id = i;
		traj_marker.markers[i].action = visualization_msgs::Marker::ADD;
		traj_marker.markers[i].pose.position.z = 0.5;
		traj_marker.markers[i].scale.x = 1;
		traj_marker.markers[i].scale.y = 0.3;
		traj_marker.markers[i].scale.z = 0.3;
		traj_marker.markers[i].color.a = 1.0;
		traj_marker.markers[i].color.r = 0.0;
		traj_marker.markers[i].color.g = 1.0;
		traj_marker.markers[i].color.b = 0.5;
		if (i != trajectory.points.size())
		{
			traj_marker.markers[i].pose.orientation.w = trajectory.points[i].transforms[0].rotation.w;
			traj_marker.markers[i].pose.orientation.z = trajectory.points[i].transforms[0].rotation.z;
			traj_marker.markers[i].pose.orientation.x = trajectory.points[i].transforms[0].rotation.x;
			traj_marker.markers[i].pose.orientation.y = trajectory.points[i].transforms[0].rotation.y;
			traj_marker.markers[i].pose.position.x = trajectory.points[i].transforms[0].translation.x;
			traj_marker.markers[i].pose.position.y = trajectory.points[i].transforms[0].translation.y;
		}
		else
		{
			traj_marker.markers[i].pose.orientation.w = goalPoseStamped.pose.orientation.w;
			traj_marker.markers[i].pose.orientation.z = goalPoseStamped.pose.orientation.z;
			traj_marker.markers[i].pose.orientation.x = goalPoseStamped.pose.orientation.x;
			traj_marker.markers[i].pose.orientation.y = goalPoseStamped.pose.orientation.y;
			traj_marker.markers[i].pose.position.x = goalPoseStamped.pose.position.x;
			traj_marker.markers[i].pose.position.y = goalPoseStamped.pose.position.y;
		}
	}
	vistraj_pub->publish(traj_marker);
	globalTrajSent = false;
}
void showTime(string message, struct timeb st, struct timeb ft)
{

	float seconds, milliseconds;

	seconds = ft.time - st.time - 1;
	milliseconds = (1000 - st.millitm) + ft.millitm;
	cout << message << (milliseconds + seconds * 1000) << " ms" << endl;
}
void configParams(bool showConfig)
{
	ros::param::get("/costmap_2d/costmap/origin_x", origin.x);
	ros::param::get("/costmap_2d/costmap/origin_y", origin.y);
	ros::param::get("/global_planner/ws_x_max", ws_x_max);
	ros::param::get("/global_planner/ws_y_max", ws_y_max);
	ros::param::get("/global_planner/ws_x_min", ws_x_min);
	ros::param::get("/global_planner/ws_y_min", ws_y_min);
	ros::param::get("/global_planner/map_resolution", map_resolution);
	ros::param::get("/global_planner/map_h_inflaction", map_h_inflaction);
	ros::param::get("/global_planner/goal_weight", goal_weight);
	ros::param::get("/global_planner/traj_dxy_max", traj_dxy_max);
	ros::param::get("/global_planner/traj_vxy_m", traj_vxy_m);
	ros::param::get("/global_planner/traj_vxy_m_1", traj_vxy_m_1);
	ros::param::get("/global_planner/traj_wyaw_m", traj_wyaw_m);
	ros::param::get("/global_planner/traj_pos_tol", traj_pos_tol);
	ros::param::get("/global_planner/traj_yaw_tol", traj_yaw_tol);

	if (showConfig)
	{
		printf("Global Planner Node Configuration:\n");
		printf("\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] \n", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
		printf("\t Map: resol.= [%.2f], inflac.= [%.2f]\n", map_resolution, map_h_inflaction);
		printf("\t Lazy Theta* with optim.: goal_weight = [%.2f]\n", goal_weight);
		printf("\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]\n", traj_dxy_max, traj_pos_tol);
		printf("\t Trajectory Cruising Speed = [%.2f] (%.2f)\n", traj_vxy_m, traj_vxy_m_1);
		printf("\t Trajectory Angular vel: [%.2f], Minimum Dpos: [%.2f]\n\n", traj_wyaw_m, traj_yaw_tol);
	}
}