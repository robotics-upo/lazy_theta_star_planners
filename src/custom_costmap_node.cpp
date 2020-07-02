#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <iostream>
#include <string>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <theta_star_2d/checkObstacles.h>
#include <theta_star_2d/addObstacle.h>

std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ptr;
typedef unsigned int uint;
int n_max;
double robot_radius;
bool use_grid_map = false;

void setObstacleCallback(const theta_star_2d::addObstacleConstPtr &obstacle_coord)
{
    if (!use_grid_map)
        return;

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr->getCostmap()->getMutex()));

    unsigned int mx, my;

    for (size_t i = 0; i < obstacle_coord->data.size() - 1; ++i)
        if (costmap_ptr->getCostmap()->worldToMap(obstacle_coord->data[i], obstacle_coord->data[i + 1], mx, my))
            costmap_ptr->getCostmap()->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
}
bool resetCostmapSrv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep)
{

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr->getCostmap()->getMutex()));
    costmap_ptr->resetLayers();
    return true;
}
bool switchInput(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &rep)
{
    //boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr->getCostmap()->getMutex()));
    //costmap_ptr->resetLayers();

    if (use_grid_map)
    {
        use_grid_map = false;
        std::system("rosrun dynamic_reconfigure dynparam set /custom_costmap_node/costmap/obstacle_layer enabled true &");
        std::system("rosrun dynamic_reconfigure dynparam set /global_planner_node/global_costmap/obstacle_layer enabled true &");
    
    }
    else
    {
        use_grid_map = true;
        std::system("rosrun dynamic_reconfigure dynparam set /custom_costmap_node/costmap/obstacle_layer enabled false &");
        std::system("rosrun dynamic_reconfigure dynparam set /global_planner_node/global_costmap/obstacle_layer enabled false &");

    }

    return true;
}
bool checkEnvSrv(theta_star_2d::checkObstaclesRequest &req, theta_star_2d::checkObstaclesResponse &rep)
{
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr->getCostmap()->getMutex()));

    static float res = costmap_ptr->getCostmap()->getResolution();

    static uint s_x = costmap_ptr->getCostmap()->getSizeInCellsX() / 2;
    static uint s_y = costmap_ptr->getCostmap()->getSizeInCellsY() / 2;
    static int robot_r_disc = static_cast<int>(robot_radius / res);

    double th;
    uint count = 0;
    int x, y;

    for (int i = -1 * robot_r_disc; i < robot_r_disc; ++i)
    {
        for (int j = -1 * robot_r_disc; j < robot_r_disc; ++j)
        {
            th = atan2(j * res, i * res);
            x = floor(s_x + robot_r_disc * cos(th));
            y = floor(s_y + robot_r_disc * sin(th));

            if (i < x && j < y && costmap_ptr->getCostmap()->getCost(i + s_x, j + s_y) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) //Check if the point is inside a circle centered at the robot
                ++count;
        }
    }

    int max_obst = req.thresh.data;
    if (max_obst == 0)
        max_obst = n_max;

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

    n.param("n_max", n_max, (int)700);
    n.param("check_radius", robot_radius, (double)0.6);
    robot_radius += 0.05;
    ros::ServiceServer reset_costmap_svr = n.advertiseService("reset_costmap", resetCostmapSrv);
    ros::ServiceServer switch_input_svr = n.advertiseService("switch_input", switchInput);

    ros::ServiceServer check_env = n.advertiseService("check_env", &checkEnvSrv);
    ros::Subscriber add_obstacle_sub = n.subscribe<theta_star_2d::addObstacle>("add_obstacle", 1, &setObstacleCallback);
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