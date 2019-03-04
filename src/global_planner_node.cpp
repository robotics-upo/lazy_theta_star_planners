#include <iostream>
#include <cstdlib>
#include <string>
#include <ctime>
#include <sys/timeb.h>
#include <math.h>
#include <ros/ros.h>
#include <fstream>
#include <theta_star/ThetaStar.hpp>
//#include <PlannersLib/PlannersLib.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#define DEBUG

using namespace std;
using namespace PathPlanners;

geometry_msgs::PoseStamped goalPoseStamped;
geometry_msgs::Vector3Stamped goal;
nav_msgs::OccupancyGrid globalCostMap;
tf2_ros::Buffer tfBuffer;

bool globalGoalReceived = false;
bool globalCostMapSentToAlgorithm = false;
bool globalCostMapReceived = false;
bool globalGoalSet = false;
bool initialPoseSet = false;
bool globalTrajSent = false;

struct timeb startT, finishT;

int number_of_points;

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

void setGoal(geometry_msgs::Vector3Stamped, ThetaStar *th);
void setStart(ThetaStar *th);

geometry_msgs::TransformStamped getTransformFromMapToThetaStarFrame();
geometry_msgs::TransformStamped getTransformFromBaseLinkToMap();
geometry_msgs::TransformStamped getTransformFromBaseLinkToThetaStarMap();
float getYawFromQuat(Quaternion quat);
void getAndPublishTrajMarkArray(ros::Publisher *traj_pub, ros::Publisher *vistraj_pub, ThetaStar *th);
void showTime(string message, struct timeb st, struct timeb ft);
void configParams(bool showConfig);

int main(int argc, char **argv)
{

	string node_name = "global_planner_node";
	ros::init(argc, argv, node_name);
	ros::NodeHandle n;

	tf2_ros::TransformListener tfListener(tfBuffer);

	char topicPath[100];

	ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1000, rvizGoalCallback);
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

	configParams(true);
	// Init Theta*
	ThetaStar theta((char *)node_name.c_str(), (char *)"/world", ws_x_max, ws_y_max, ws_x_min, ws_y_min, map_resolution, map_h_inflaction, goal_weight, &n);

	// Set timeout
	theta.setTimeOut(20);

	// Set resulting trajectroy parameters
	theta.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_vxy_m, traj_vxy_m_1, traj_wyaw_m, traj_yaw_tol);

	ROS_INFO("Waiting for new global goal...");

	ros::Rate loop_rate(10);
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

		if (globalGoalReceived && globalCostMapSentToAlgorithm)
		{

			setGoal(goal, &theta);
			setStart(&theta);
			if (globalGoalSet && initialPoseSet)
			{
				// Path calculation
				ROS_INFO("Path calculation...");

				ftime(&startT);
				number_of_points = theta.computePath();
				ftime(&finishT);


				showTime("Time spend in path calculation: ", startT, finishT);
				if(number_of_points >0){
				ROS_INFO("Number of points: %d", number_of_points);

				getAndPublishTrajMarkArray(&trajectory_pub, &vis_pub_traj, &theta);
				}
				globalGoalReceived = false;
			}else{
				ROS_ERROR("Global goal or start poitn occupied ");
			}
			
		}
		loop_rate.sleep();
	}

	return 0;
}
//Since the local costmap y constant over time, it's not neccesary to refresh it every ros::spinOnce
void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &fp)
{
	if (!globalCostMapReceived)
	{
		globalCostMap = *fp;
		globalCostMapReceived = true;
	}
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
}
//This function obtains the transform between map and base_link to send it to the trayectory calculation function from ThetaStar class
geometry_msgs::TransformStamped getTransformFromBaseLinkToMap()
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
geometry_msgs::TransformStamped getTransformFromBaseLinkToThetaStarMap()
{

	geometry_msgs::TransformStamped ret;

	try
	{
		ret = tfBuffer.lookupTransform("global_theta_star_link", "base_link", ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
	}

	return ret;
}
geometry_msgs::TransformStamped getTransformFromMapToThetaStarFrame()
{
	geometry_msgs::TransformStamped ret;

	try
	{
		ret = tfBuffer.lookupTransform("global_theta_star_link", "map", ros::Time(0));
	}
	catch (tf::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
	}

	return ret;
}
void setGoal(geometry_msgs::Vector3Stamped goal, ThetaStar *th)
{

	goal.vector.x += getTransformFromMapToThetaStarFrame().transform.translation.x;
	goal.vector.y += getTransformFromMapToThetaStarFrame().transform.translation.y;

	if (th->setValidFinalPosition(goal.vector))
	{
		globalGoalSet = true;
	}
	else
	{
		ROS_WARN("Failed to set final global position");
	}
}
void setStart(ThetaStar *th)
{

	geometry_msgs::Vector3Stamped start;
	start.vector.x = getTransformFromBaseLinkToThetaStarMap().transform.translation.x;
	start.vector.y = getTransformFromBaseLinkToThetaStarMap().transform.translation.y;

	if (th->setValidInitialPosition(start.vector))
	{
		initialPoseSet = true;
	}
	else
	{
		ROS_WARN("Failed to set initial global position");
	}
}
float getYawFromQuat(Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y / M_PI * 180;
}
void getAndPublishTrajMarkArray(ros::Publisher *traj_pub, ros::Publisher *vistraj_pub, ThetaStar *th)
{

	Trajectory trajectory;
	visualization_msgs::MarkerArray traj_marker;

	trajectory.joint_names.push_back("map");
	trajectory.header.stamp = ros::Time::now();
	trajectory.header.frame_id = "map";

	trajectory.points.clear();
	trajectory.header.stamp = ros::Time::now();
	ROS_INFO("Trajectory calculation...");

	//This way we get the rob0000000009ot pose in the ThetaStar map frame( origin(0,0) in the lower left corner)
	geometry_msgs::TransformStamped transform_robot_pose = getTransformFromBaseLinkToThetaStarMap();
	if(number_of_points>1){
    th->getTrajectoryYawInAdvance(trajectory, transform_robot_pose.transform);
    }else{
        th->getTrajectoryYawFixed(trajectory,getYawFromQuat(transform_robot_pose.transform.rotation));
        trajectory_msgs::MultiDOFJointTrajectoryPoint pos;
        pos.transforms.push_back(transform_robot_pose.transform);   
        trajectory.points.push_back(pos);
    }
	// Send the trajectory
	// Trajectory solution visualization marker

	for (int i = 0; i < trajectory.points.size(); i++)
	{
		trajectory.points[i].transforms[0].translation.x -= getTransformFromMapToThetaStarFrame().transform.translation.x;
		trajectory.points[i].transforms[0].translation.y -= getTransformFromMapToThetaStarFrame().transform.translation.y;
	}
	trajectory_msgs::MultiDOFJointTrajectoryPoint goal_multidof;
	geometry_msgs::Transform transform_goal;

	transform_goal.rotation = goalPoseStamped.pose.orientation;
	transform_goal.translation.x = goalPoseStamped.pose.position.x;
	transform_goal.translation.y = goalPoseStamped.pose.position.y;

	goal_multidof.transforms.resize(1, transform_goal);

	trajectory.points.push_back(goal_multidof);

	

	traj_pub->publish(trajectory);

	traj_marker.markers.resize( trajectory.points.size());

	for (int i = 0; i < trajectory.points.size(); i++)
	{
		traj_marker.markers[i].type = visualization_msgs::Marker::CUBE;
		traj_marker.markers[i].points.clear();
		traj_marker.markers[i].header.frame_id = "map";
		traj_marker.markers[i].header.stamp = ros::Time();
		traj_marker.markers[i].ns = "theta_star";
		traj_marker.markers[i].id = i;
		traj_marker.markers[i].action = visualization_msgs::Marker::ADD;
		traj_marker.markers[i].pose.position.z = 0.3;
		traj_marker.markers[i].scale.x = 0.3;
		traj_marker.markers[i].scale.y = 0.3;
		traj_marker.markers[i].scale.z = 0.3;
		traj_marker.markers[i].color.a = 1.0;
		traj_marker.markers[i].color.r = 0.0;
		traj_marker.markers[i].color.g = 1.0;
		traj_marker.markers[i].color.b = 0.5;
		traj_marker.markers[i].lifetime = ros::Duration(60);
		traj_marker.markers[i].pose.orientation.w = trajectory.points[i].transforms[0].rotation.w;
		traj_marker.markers[i].pose.orientation.z = trajectory.points[i].transforms[0].rotation.z;
		traj_marker.markers[i].pose.orientation.x = trajectory.points[i].transforms[0].rotation.x;
		traj_marker.markers[i].pose.orientation.y = trajectory.points[i].transforms[0].rotation.y;
		traj_marker.markers[i].pose.position.x = trajectory.points[i].transforms[0].translation.x;
		traj_marker.markers[i].pose.position.y = trajectory.points[i].transforms[0].translation.y;
	}
	vistraj_pub->publish(traj_marker);
	globalTrajSent = true;
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
