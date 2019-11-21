#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>
#include <string>

#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ptr;
typedef unsigned int uint;
double x_b,y_b,n_max;
std::string robot_frame;
visualization_msgs::Marker markers;

bool resetCostmapSrv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep){

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr->getCostmap()->getMutex()));
    costmap_ptr->resetLayers();
    return true;
}
bool checkEnvSrv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep){

    static float res=costmap_ptr->getCostmap()->getResolution();
    uint count=0;

    static uint s_x=costmap_ptr->getCostmap()->getSizeInCellsX()/2;
    static uint s_y=costmap_ptr->getCostmap()->getSizeInCellsY()/2;

    static uint x1=static_cast<uint>(x_b/res);
    static uint x2=static_cast<uint>(x_b/res);
    static uint y1=static_cast<uint>(y_b/res);
    static uint y2=static_cast<uint>(y_b/res);

    for(uint i = (s_x-x1); i < (s_x+x2); ++i)
        for(uint j = (s_y-y1); j < (s_y+y2); ++j)
            if(costmap_ptr->getCostmap()->getCost(i,j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                ++count;
    ROS_INFO("Sx: %d\t Sy: %d\t x1: %d\t x2: %d\t y1: %d\t y2: %d", s_x,s_y,x1,x2,y1,y2);
    if(count > n_max){
        rep.message="Too much obstacles: " + std::to_string(count);
        rep.success=false;
    }else{
        rep.message="Okey, only "+std::to_string(count)+" obstacles found";
        rep.success=true;
    }

    return true;
}
void configMarkers(){

   markers.points.resize(0);
    
    markers.header.frame_id = robot_frame;
    markers.header.stamp = ros::Time();
    markers.ns = "local_costmap";
    markers.id = 1;
    markers.type = visualization_msgs::Marker::LINE_STRIP;
    markers.action = visualization_msgs::Marker::ADD;
    markers.lifetime = ros::Duration(100000);
    markers.scale.x = 0.1;
    markers.color.a = 1.0;
    markers.color.b = 1.0;
    markers.color.g = 1.0;
    markers.color.r = 0.0;
    markers.pose.orientation.w=1;

    geometry_msgs::Point p;
    p.x = x_b;
    p.y = y_b;
    markers.points.push_back(p);
    p.y=-y_b;
    markers.points.push_back(p);
    p.x=-x_b;
    p.y=-y_b;
    markers.points.push_back(p);
    p.y=y_b;
    markers.points.push_back(p);
    p.x = x_b;
    markers.points.push_back(p);
}
int main(int argc, char** argv){

    
    ros::init(argc,argv,"custom_costmap_node");
    ros::NodeHandle n("~");
    
    n.param("x_bound", x_b,(double)0.65);
    n.param("y_bound", y_b,(double)0.4);
    n.param("n_max", n_max, (double)20);
    n.param("robot_frame", robot_frame, (std::string)"siar/base_link");
    
    ros::ServiceServer reset_costmap_svr = n.advertiseService("reset_costmap", resetCostmapSrv);
    ros::ServiceServer check_env = n.advertiseService("check_env", &checkEnvSrv);
    ros::Publisher markers_pub = n.advertise<visualization_msgs::Marker>("rotation_check_markers", 2);
    configMarkers();
    
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
    while(ros::ok()){
        ros::spinOnce();
        if( markers_pub.getNumSubscribers() > 0){
            markers_pub.publish(markers);
        }
        loop_rate.sleep();
    }
   
    
    return 0;
}