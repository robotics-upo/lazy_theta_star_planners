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
#include <tf2_ros/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/simConfig.h>

//#define DEBUG

using namespace std;
using namespace PathPlanners;

visualization_msgs::Marker markerTraj;
geometry_msgs::Vector3 start, goal;
nav_msgs::OccupancyGrid globalCostMap, mp;

nav_msgs::OccupancyGrid::ConstPtr gCmPtr;

tf2_ros::Buffer tfBuffer;

bool globalGoalReceived = false;
bool globalCostMapSentToAlgorithm = false;
bool globalCostMapReceived = false;
bool globalGoalSet = false;
bool initialPoseSet = false;
bool globalTrajSent = false;

struct timeb startT, finishT;
int it;
int number_of_points;
float t_time = 0;

double map_resolution = 0.0;
double ws_x_max = 0.0;
double ws_y_max = 0.0;
double ws_x_min = 0.0;
double ws_y_min = 0.0;

double cost_weight = 0.0;
double goal_weight = 1.0;
double lof_distance = 1.0;
int occ_threshold = 80.0;
double traj_dxy_max = 0.5;
double traj_pos_tol = 1.0;
double traj_yaw_tol = 1.0;

void callback(theta_star_2d::simConfig &config, uint32_t level);
void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &fp);
void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
void rvizStartCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &goalMsg);

void setGoal(ThetaStar *th);
void setStart(ThetaStar *th);

float getAndPublishTrajMarkArray(ros::Publisher *traj_pub, ros::Publisher *vistraj_pub, ThetaStar *th);
void showTime(string message, struct timeb st, struct timeb ft);
void configParams(bool showConfig);

int main(int argc, char **argv)
{

	string node_name = "sim_planner_node";
	ros::init(argc, argv, node_name);
	ros::NodeHandle n;

	tf2_ros::TransformListener tfListener(tfBuffer);

	char topicPath[100];

	ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1000, rvizGoalCallback);
	ros::Subscriber start_sub = n.subscribe("/initialpose", 1, rvizStartCallback);

	// Global costmap topic subscriber
	sprintf(topicPath, "/costmap_2d/costmap/costmap");
	ros::Subscriber global_sub_map = n.subscribe(topicPath, 0, globalCostMapCallback);
	ROS_INFO("Theta Star: Global costmap input topic: %s", topicPath);

	// Trajectory list topic publisher to trajectory_tracker_node
	sprintf(topicPath, "trajectory_tracker/input_trajectory");
	ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 10);
	ROS_INFO("Theta Star: trajectory output topic: %s", topicPath);

	// Trajectory solution visualization topic
	ros::Publisher vis_pub_traj = n.advertise<visualization_msgs::Marker>(node_name + "/visualization_marker_trajectory", 10);

	//Dynamic reconfigure
	dynamic_reconfigure::Server<theta_star_2d::simConfig> server;
	dynamic_reconfigure::Server<theta_star_2d::simConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	//

	configParams(true);

	ThetaStar theta((char *)node_name.c_str(), (char *)"/map", ws_x_max, ws_y_max, ws_x_min, ws_y_min, map_resolution, goal_weight, cost_weight, lof_distance, occ_threshold, &n);
	theta.setTimeOut(2000);
	theta.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);
	theta.confPrintRosWarn(true);
	theta.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);

	//ros::Subscriber map_server_data = n.subscribe("/map_metadata",1,&ThetaStar::mapMdCb, &theta);
	float seconds, milliseconds;

	ROS_INFO("Waiting for new global goal...");
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	ros::Rate loop_rate(1);
	ofstream file;
	file.open("/home/fali-srl/Documentos/tfg/resultados/data_lazy_mod/results.txt", ios::out);
	
	while (ros::ok())
	{
		// Waiting for new goal and reading odometry
		ros::spinOnce();

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "map";
		transformStamped.child_frame_id = "base_link";
		transformStamped.transform.translation.x = start.x;
		transformStamped.transform.translation.y = start.y;
		transformStamped.transform.translation.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, 0);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();

		br.sendTransform(transformStamped);

		if (globalCostMapReceived && !globalCostMapSentToAlgorithm)
		{
			ftime(&startT);
			theta.getMap(gCmPtr);
			ftime(&finishT);
			seconds = finishT.time - startT.time - 1;
			milliseconds = (1000 - startT.millitm) + finishT.millitm;
			//ROS_INFO("%.2f", milliseconds+seconds*1000);
			//ROS_INFO("Global Planner: Global map received");
			globalCostMapSentToAlgorithm = true;
		}

		if (globalGoalReceived && globalCostMapSentToAlgorithm)
		{
			setGoal(&theta);
			setStart(&theta);

			if (globalGoalSet && initialPoseSet)
			{
				configParams(false);
				theta.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);
				// Path calculation
				ROS_INFO("Path calculation...");

				int steps = 1;
				//cost_weight = 0;
				//lof_distance = 0.5;
				float length = 0;
				int expanded_nodes = 0;
				//for (int j = 0; j < 10; j++)
				//{
					//file << "Test lof "<<j*0.5<<"\n";
					for (int i = 0; i < steps; i++)
					{
						t_time = 0;
						expanded_nodes = 0;
						theta.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);

						for (int i = 0; i < it; i++)
						{
							ftime(&startT);
							number_of_points = theta.computePath();
							ftime(&finishT);
							seconds = finishT.time - startT.time - 1;
							milliseconds = (1000 - startT.millitm) + finishT.millitm;
							t_time += milliseconds + seconds * 1000;
							//expanded_nodes += theta.getExpNodesNumber();
							//ROS_INFO("%.2f", t_time);
							//showTime("Tiempo en calcular:", startT, finishT);

							if (number_of_points > 0)
							{
								//ROS_INFO("Number of points: %d", number_of_points);
								length = getAndPublishTrajMarkArray(&trajectory_pub, &vis_pub_traj, &theta);
							}
						}
						//cost_weight+=0.01;
						//file << lof_distance << "\t" << t_time / it << "\t" << expanded_nodes / it << "\t" << length << "\n";
						ROS_INFO("Tiempo promedio: %.4f", t_time / it);
						//cost_weight += 0.05;
					}
					file<<"\n";
					//cost_weight = 0;
					//lof_distance += 0.5;
					theta.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);
				//}
			}
			file.close();
			globalGoalReceived = false;
		}
		globalCostMapSentToAlgorithm = false;
		loop_rate.sleep();
	}

	return 0;
}
void callback(theta_star_2d::simConfig &config, uint32_t level)
{
	ROS_INFO("Dynamic Reconfigure Params Received");
}
//Since the local costmap y constant over time, it's not neccesary to refresh it every ros::spinOnce
void globalCostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &fp)
{
	gCmPtr = fp;
	globalCostMap = *fp;
	globalCostMapReceived = true;
}
void rvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goalMsg)
{
	goal.x = goalMsg->pose.position.x;
	goal.y = goalMsg->pose.position.y;
	globalGoalReceived = true;
}
void rvizStartCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &startMsg)
{
	start.x = startMsg->pose.pose.position.x;
	start.y = startMsg->pose.pose.position.y;
}
void setGoal(ThetaStar *th)
{
	if (th->setValidFinalPosition(goal))
	{
		globalGoalSet = true;
	}
	else
	{
		ROS_WARN("Sim Planner:  Failed to set final global position");
	}
}
void setStart(ThetaStar *th)
{
	if (th->setValidInitialPosition(start))
	{
		initialPoseSet = true;
	}
	else
	{
		ROS_WARN("Sim Planner: Failed to set initial global position");
	}
}
float getAndPublishTrajMarkArray(ros::Publisher *traj_pub, ros::Publisher *vistraj_pub, ThetaStar *th)
{
	Trajectory trajectory;

	trajectory.joint_names.push_back("map");
	trajectory.header.stamp = ros::Time::now();
	trajectory.header.frame_id = "map";

	trajectory.points.clear();
	trajectory.header.stamp = ros::Time::now();
	//ROS_INFO("Trajectory calculation...");

	geometry_msgs::TransformStamped transform_robot_pose;

	transform_robot_pose.transform.translation.x = start.x;
	transform_robot_pose.transform.translation.y = start.y;
	transform_robot_pose.transform.rotation.z = 0;
	transform_robot_pose.transform.rotation.w = 1;
	th->getTrajectoryYawFixed(trajectory, 0);

	trajectory_msgs::MultiDOFJointTrajectoryPoint goal_multidof, start_tp;
	geometry_msgs::Transform transform_goal;

	transform_goal.rotation.w = 1;
	transform_goal.rotation.z = 1;
	transform_goal.translation.x = goal.x;
	transform_goal.translation.y = goal.y;
	goal_multidof.transforms.resize(1, transform_goal);
	start_tp.transforms.resize(1, transform_robot_pose.transform);

	trajectory.points.push_back(goal_multidof);
	trajectory.points.insert(trajectory.points.begin(), start_tp);

	traj_pub->publish(trajectory);

	markerTraj.points.clear();

	geometry_msgs::Point p, prev;
	float path_length = 0;
	prev.x = 0;
	prev.y = 0;
	for (int i = 0; i < trajectory.points.size(); i++)
	{

		p.x = trajectory.points[i].transforms[0].translation.x;
		p.y = trajectory.points[i].transforms[0].translation.y;
		markerTraj.points.push_back(p);
	}
	//Calculo de longitud total
	int t_s = trajectory.points.size();
	for (int i = 1; i < t_s; i++)
	{
		prev.x = trajectory.points[i - 1].transforms[0].translation.x;
		prev.y = trajectory.points[i - 1].transforms[0].translation.y;

		p.x = trajectory.points[i].transforms[0].translation.x;
		p.y = trajectory.points[i].transforms[0].translation.y;

		path_length += sqrtf(pow(prev.x - p.x, 2) + pow(prev.y - p.y, 2));
	}
	ROS_INFO(PRINTF_YELLOW "Path Length: %.2f, Ave dist2obs: %.2f", path_length, th->getAvDist2Obs());

	markerTraj.type = RVizMarker::CUBE_LIST;
	markerTraj.id = 12221;
	markerTraj.color.r = 1.0;
	markerTraj.color.g = 0.0;
	markerTraj.color.b = 0.0;
	markerTraj.scale.x = 10.0 * map_resolution;
	markerTraj.scale.y = 10.0 * map_resolution;
	vistraj_pub->publish(markerTraj);

	markerTraj.type = RVizMarker::LINE_STRIP;
	markerTraj.scale.x = 2.0 * map_resolution;
	markerTraj.id = 12222;
	markerTraj.color.r = 0.0;
	markerTraj.color.g = 0.0;
	markerTraj.color.b = 1.0;

	vistraj_pub->publish(markerTraj);

	globalTrajSent = true;
	return path_length;
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
	ros::param::get("/sim_planner_node/ws_x_max", ws_x_max);
	ros::param::get("/sim_planner_node/ws_y_max", ws_y_max);
	ros::param::get("/sim_planner_node/ws_x_min", ws_x_min);
	ros::param::get("/sim_planner_node/ws_y_min", ws_y_min);
	ros::param::get("/sim_planner_node/map_resolution", map_resolution);
	ros::param::get("/sim_planner_node/goal_weight", goal_weight);
	ros::param::get("/sim_planner_node/cost_weight", cost_weight);
	ros::param::get("/sim_planner_node/traj_dxy_max", traj_dxy_max);
	ros::param::get("/sim_planner_node/traj_pos_tol", traj_pos_tol);
	ros::param::get("/sim_planner_node/traj_yaw_tol", traj_yaw_tol);
	ros::param::get("/sim_planner_node/lof_distance", lof_distance);
	ros::param::get("/sim_planner_node/occ_threshold", occ_threshold);
	ros::param::get("/sim_planner_node/it", it);
	if (showConfig)
	{
		printf("Sim Planner Node Configuration:\n");
		printf("\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] \n", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
		printf("\t Map: resol.= [%.2f]\n", map_resolution);
		printf("\t Lazy Theta* with optim.: goal_weight = [%.2f]\n", goal_weight);
		printf("\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]\n", traj_dxy_max, traj_pos_tol);
		printf("\t Goal and Cost weights: [%.2f, %.2f]\n", goal_weight, cost_weight);
		printf("\t Line of Sight distance restriction: %.2f\n", lof_distance);
		printf("\t Occupied Threshold: %d\n", occ_threshold);
		printf("\t Iterations: %d\n", it);
	}
	markerTraj.header.frame_id = "map";
	markerTraj.header.stamp = ros::Time();
	markerTraj.ns = "sim_path";
	markerTraj.id = 12221;
	markerTraj.type = RVizMarker::CUBE_LIST;
	markerTraj.action = RVizMarker::ADD;
	markerTraj.pose.orientation.w = 1.0;
	markerTraj.lifetime = ros::Duration(1000);
	markerTraj.scale.x = 3.0 * map_resolution;
	markerTraj.scale.y = 3.0 * map_resolution;
	markerTraj.pose.position.z = 0;
	markerTraj.color.a = 1.0;
	markerTraj.color.r = 1.0;
	markerTraj.color.g = 0.0;
	markerTraj.color.b = 0.0;
}