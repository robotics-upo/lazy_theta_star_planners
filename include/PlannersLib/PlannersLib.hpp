#ifndef PLANNERSLIB_H_
#define PLANNERSLIB_H_

/* Common functions used by local planner, global planner and other modules


*/

#include <iostream>
#include <cstdlib>
#include <string>
#include <ctime>
#include <sys/timeb.h>
#include <math.h>
#include <ros/ros.h>
#include <fstream>
#include <theta_star/ThetaStar.hpp>

#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void showTime(string message, struct timeb st, struct timeb ft)
{

	float seconds, milliseconds;

	seconds = ft.time - st.time - 1;
	milliseconds = (1000 - st.millitm) + ft.millitm;
	cout << message << (milliseconds + seconds * 1000) << " ms" << endl;
}
float getYawFromQuat(geometry_msgs::Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y / M_PI * 180;
}




#endif