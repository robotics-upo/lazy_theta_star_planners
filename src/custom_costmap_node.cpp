#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>
#include <string>

#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

//#define MELODIC 

std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ptr;

bool resetCostmapSrv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep){

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr->getCostmap()->getMutex()));
    costmap_ptr->resetLayers();
    return true;
}
int main(int argc, char** argv){

    
    ros::init(argc,argv,"custom_costmap_node");
    ros::NodeHandle n("~");
    ros::ServiceServer reset_costmap_svr = n.advertiseService("reset_costmap", resetCostmapSrv);

    #ifdef MELODIC
    tf2_ros::Buffer buffer(ros::Duration(5));
    tf2_ros::TransformListener tf(buffer);
    costmap_ptr.reset(new costmap_2d::Costmap2DROS("costmap", buffer));
    #endif
    #ifndef MELODIC
    tf::TransformListener tfListener(ros::Duration(5));    
    costmap_ptr.reset(new costmap_2d::Costmap2DROS("costmap", tfListener));
    #endif

    ros::spin();
    
    return 0;
}