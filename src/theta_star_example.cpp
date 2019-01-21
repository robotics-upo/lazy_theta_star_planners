/*
 * Copyright 2015 Ricardo Ragel de la Torre, GRVC, Univ. of Seville, Spain
 *
 * Resume: Lazy Theta Star with Optimization Example using Octomap to build the Occupancy Map
 * 
 * Input: /theta_star/goal_position (XYZ goal to plan from the current position)
 * 
 * Output: /trajectory_tracking/trajectory_input ([XYZT] vector to the trajectory tracker)
 * 
 * NOTE: For get rviz markers debug visualization go to 'theta_star/src/ThetaStar.cpp' and un-comment the line '#define DEBUG'
 */

#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <theta_star/ThetaStar.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <ctime>
#include <sys/timeb.h>

#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace PathPlanners;

// Odometry message callback. Simply remap the pose data in the odom_pose var.
geometry_msgs::Transform odom_pose;

// Odometry message callback. Simply remap the octomap msg
nav_msgs::OccupancyGrid m;
bool map_received = false;
void CollisionMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& fp)
{
    m = *fp;
    map_received = true;
}

double map_resolution = 0.0;
geometry_msgs::Vector3 goal,init;
bool goal_received = true;
//The inputs x_ etc are pixels(more confortable sometimes)
void setGoal(float x_, float y_, float z_){
	goal.x = x_*map_resolution;
	goal.y = y_*map_resolution;
	goal.z = z_*map_resolution;
}
void setPose(float x_, float y_, float z_, float xw_, float yw_, float zw_, float w_){
	odom_pose.translation.x = x_;
  	odom_pose.translation.y = y_;
  	odom_pose.translation.z = z_;
  	odom_pose.rotation.x = xw_;
	odom_pose.rotation.y = yw_;
	odom_pose.rotation.z = zw_;
	odom_pose.rotation.w = w_;
}
//The inputs x_ etc are pixels(more confortable sometimes)
void setIni(float x_, float y_, float z_){
	
	init.x = x_*map_resolution;
	init.y = y_*map_resolution;
	init.z = z_*map_resolution;
}

int main(int argc, char **argv)
{
    struct timeb startT, finishT;
    float seconds, milliseconds;

	string node_name = "theta_start_example";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

	// Odometry data topic subscriber
	char topicPath[100];
	
	// Trajectory list topic publisher to trajectory_tracker_node
	sprintf(topicPath, "trajectory_tracker/input_trajectory");
	ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 10);
	ROS_INFO("Theta Star Example: trajectory output topic: %s", topicPath);

	// Path solution visualization topic
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>(node_name + "/visualization_marker_path", 10 );

	// Trajectory solution visualization topic
    ros::Publisher vis_pub_traj = n.advertise<visualization_msgs::Marker>(node_name + "/visualization_marker_trajectory", 10 );

	// Map server topic subscriber
	sprintf(topicPath, "map");
    ros::Subscriber sub_map = n.subscribe(topicPath, 0, CollisionMapCallBack);
    ROS_INFO("Theta Star Example: map input topic: %s", topicPath);


	// Read parameters
	double ws_x_max = 0.0;
	double ws_y_max = 0.0;
	double ws_z_max = 0.0;
	double ws_x_min = 0.0;
	double ws_y_min = 0.0;
	
	double map_h_inflaction = 0.0;
	double goal_weight = 1.0;
	double traj_dxy_max = 1.0;
	double traj_vxy_m = 1.0;
	double traj_vxy_m_1 = 1.0;
	double traj_wyaw_m = 1.0;
	double traj_pos_tol = 1.0;
	double traj_yaw_tol = 1.0;
	int ixp,iyp,gxp,gyp;

	ros::param::get("/theta_star/start_x", ixp);
	ros::param::get("/theta_star/start_y", iyp);
	ros::param::get("/theta_star/goal_x", gxp);
	ros::param::get("/theta_star/goal_y", gyp);
    ros::param::get("/theta_star/ws_x_max", ws_x_max);	
    ros::param::get("/theta_star/ws_y_max", ws_y_max);	
    ros::param::get("/theta_star/ws_x_min", ws_x_min);	
    ros::param::get("/theta_star/ws_y_min", ws_y_min);	
    ros::param::get("/theta_star/map_resolution", map_resolution);	
    ros::param::get("/theta_star/map_h_inflaction", map_h_inflaction);	
    ros::param::get("/theta_star/goal_weight", goal_weight);
    ros::param::get("/theta_star/traj_dxy_max", traj_dxy_max);
    ros::param::get("/theta_star/traj_vxy_m", 	traj_vxy_m);
    ros::param::get("/theta_star/traj_vxy_m_1", traj_vxy_m_1);
    ros::param::get("/theta_star/traj_wyaw_m", 	traj_wyaw_m);
    ros::param::get("/theta_star/traj_pos_tol", 	traj_pos_tol);
    ros::param::get("/theta_star/traj_yaw_tol", 	traj_yaw_tol);
    
    // Debug
    printf("Theta Star Configuration:\n");
    printf("\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] \n", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
    printf("\t Map: resol.= [%.2f], inflac.= [%.2f]\n", map_resolution, map_h_inflaction);
    printf("\t Lazy Theta* with optim.: goal_weight = [%.2f]\n", goal_weight);
    printf("\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]\n", traj_dxy_max, traj_pos_tol);
    printf("\t Trajectory Cruising Speed = [%.2f] (%.2f)\n", traj_vxy_m, traj_vxy_m_1);
    printf("\t Trajectory Angular vel: [%.2f], Minimum Dpos: [%.2f]\n\n", traj_wyaw_m, traj_yaw_tol);
    
    // Init Theta*
    ThetaStar theta((char*)node_name.c_str(), (char*)"/world", ws_x_max, ws_y_max, ws_x_min, ws_y_min, map_resolution, map_h_inflaction, goal_weight, &n);
	
	// Set timeout
	theta.setTimeOut(20);
	
	// Set resulting trajectroy parameters
	theta.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_vxy_m, traj_vxy_m_1, traj_wyaw_m, traj_yaw_tol);
	
	// Path solution visualization marker
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time();
    path_marker.ns = "theta_star";
    path_marker.id = 1;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.1;
    path_marker.scale.y = 0.1;

    path_marker.color.a = 1.0;
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;
    
    // Trajectory solution visualization marker
    visualization_msgs::Marker traj_marker;
	traj_marker.points.clear();
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = ros::Time();
    traj_marker.ns = "theta_star";
    traj_marker.id = 2;
    traj_marker.type = visualization_msgs::Marker::CUBE_LIST;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = 0.1;
    traj_marker.scale.y = 0.1;
    traj_marker.color.a = 1.0;
    traj_marker.color.r = 1.0;
    traj_marker.color.g = 0.0;
    traj_marker.color.b = 0.5;    

	
	seconds = finishT.time - startT.time - 1;
	milliseconds = (1000 - startT.millitm) + finishT.millitm;
	cout << "Time spend in construct the occupancy map: " << (milliseconds + seconds * 1000) << " ms" << endl;

	// Start and Goal positions
	Vector3 fin;

	// Result Trajectory
	Trajectory trajectory;
	trajectory.joint_names.push_back("base_link");  
	trajectory.header.stamp = ros::Time::now();
	trajectory.header.frame_id = "base_link";
	

	int countToPublishOccMap = 0;
	
	//Save path points coordinates to file
	ofstream file("/home/fali/catkin_ws_ts/src/theta_star/resource/output_path.txt");
	
	
	ROS_INFO("Waiting for new goal...");
	bool map_created = false;
	setGoal(gxp,gyp,0);
	setIni(ixp,iyp,0);
	while(ros::ok())
	{
		// Waiting for new goal and reading odometry
		ros::spinOnce();
		if(map_received && !map_created){
			ROS_INFO("Map received");
			theta.getMap(m);
			map_created = true;
		}
		
		
		if(goal_received && map_created){
			// Set initial and final position to theta* and calculate the trajectory only if these are valid positions
			if(theta.setValidInitialPosition(init) && theta.setValidFinalPosition(goal))
			{
				// Path calculation
				ROS_INFO("Path calculation...");
				ftime(&startT);
					int number_of_points = theta.computePath();
				ftime(&finishT);

				ROS_INFO("Number of points: %d", number_of_points);

				seconds = finishT.time - startT.time - 1;
				milliseconds = (1000 - startT.millitm) + finishT.millitm;
				cout << "Time spend in path calculation: " << (milliseconds + seconds * 1000) << " ms" << endl;

				// Get, draw and compute length the new path
				path_marker.points.clear();
				path_marker.header.stamp = ros::Time();

				geometry_msgs::Point pIni;
				pIni.x = init.x;
				pIni.y = init.y;
				pIni.z = init.z;
				path_marker.points.push_back(pIni);

				float total_distance = 0.0;
				float difx,dify;

				file<<init.x/map_resolution+1<<" "<<init.y/map_resolution+1<<endl;
				
				geometry_msgs::Point p;

				for(unsigned int i = 0; i < theta.getCurrentPath().size();i++)
				{
					p.x = theta.getCurrentPath()[i].x;
					p.y = theta.getCurrentPath()[i].y;
					
					if(i == 0){
						difx = pIni.x - p.x;
						dify = pIni.y - p.y;
					}else{
						difx = (theta.getCurrentPath()[i-1].x - p.x);
						dify = (theta.getCurrentPath()[i-1].y - p.y);
					}

					total_distance += sqrtf( difx*difx+dify*dify );

					//ROS_WARN("Nodo %d : [%f, %f] ", i, p.x/map_resolution,p.y/map_resolution);
					//Save point coordinate to file
					file<<p.x/map_resolution+1<<" "<<p.y/map_resolution+1<<endl;
					path_marker.points.push_back(p);
				}
				file.close();
				
				vis_pub.publish(path_marker);
				ROS_INFO("Path Length: %.4f", total_distance);
				
				/* Get the trajectory
				trajectory.points.clear();
				trajectory.header.stamp = ros::Time::now();
				ROS_INFO("Trajectory calculation...");
				
				setPose(0,0,0,0,0,0,1);
				theta.getTrajectoryYawInAdvance(trajectory, odom_pose);

				// Send the trajectory
				trajectory_pub.publish(trajectory);
				*/
				
			}
			else
				ROS_ERROR("Initial or Final position outside the WS or occupied\n");
			
			goal_received = false;
			ROS_INFO("Waiting for new goal...");
		}
		
	    usleep(1e5);
	}
	
    return 0;
}
