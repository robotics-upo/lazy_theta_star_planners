#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>
#include <string>

#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <theta_star_2d/checkObstacles.h>

std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ptr;
typedef unsigned int uint;
int n_max;
double robot_radius;

bool resetCostmapSrv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep)
{

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr->getCostmap()->getMutex()));
    costmap_ptr->resetLayers();
    return true;
}
bool checkEnvSrv(theta_star_2d::checkObstaclesRequest &req, theta_star_2d::checkObstaclesResponse &rep)
{
    // boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr->getCostmap()->getMutex()));
    // costmap_ptr->resetLayers();
    // lock.unlock();
    
    static float res = costmap_ptr->getCostmap()->getResolution();

    static uint s_x = costmap_ptr->getCostmap()->getSizeInCellsX() / 2;
    static uint s_y = costmap_ptr->getCostmap()->getSizeInCellsY() / 2;
    static int robot_r_disc = static_cast<int>(robot_radius/res);
    
    double th;
    uint count = 0;
    int x, y;
    
    for (int i = -1*robot_r_disc; i < robot_r_disc ; ++i)
    {
        for (int j = -1*robot_r_disc; j < robot_r_disc ; ++j)
        {
            th = atan2(j * res, i * res);
            x = floor(s_x + robot_r_disc * cos(th));
            y = floor(s_y + robot_r_disc * sin(th));

            if (i < x && j < y && costmap_ptr->getCostmap()->getCost(i+s_x, j+s_y) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) //Check if the point is inside a circle centered at the robot
                ++count;
        }
    }

    int max_obst = req.thresh.data;
    if( max_obst == 0)
        max_obst=n_max;

        
    if (count > max_obst)
    {
        rep.message = "Too much obstacles: " + std::to_string(count);
        rep.success = false;
    }
    else
    {
        rep.message = "Okey, only " + std::to_string(count) + " obstacles found";
        rep.success = true;
    }

    return true;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "custom_costmap_node");
    ros::NodeHandle n("~");

    n.param("n_max", n_max, (int)100);
    n.param("costmap/robot_radius", robot_radius, (double)0.35);
    robot_radius+=0.05;
    ros::ServiceServer reset_costmap_svr = n.advertiseService("reset_costmap", resetCostmapSrv);
    ros::ServiceServer check_env = n.advertiseService("check_env", &checkEnvSrv);

#ifdef MELODIC
    tf2_ros::Buffer buffer(ros::Duration(5));
    tf2_ros::TransformListener tf(buffer);
    costmap_ptr.reset(new costmap_2d::Costmap2DROS("costmap", buffer));
#endif
#ifndef MELODIC
    tf::TransformListener tfListener(ros::Duration(5));
    costmap_ptr.reset(new costmap_2d::Costmap2DROS("costmap", tfListener));
#endif
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}