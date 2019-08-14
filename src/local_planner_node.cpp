#include <theta_star/LocalPlanner.hpp>


using namespace PathPlanners;


int main(int argc, char **argv)
{

	string node_name = "local_planner_node";
	ros::init(argc, argv, node_name);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    LocalPlanner lcPlanner(&tfBuffer);
    dynamic_reconfigure::Server<theta_star_2d::localPlannerConfig> server;
  	dynamic_reconfigure::Server<theta_star_2d::localPlannerConfig>::CallbackType f;

  	f = boost::bind(&LocalPlanner::dynRecCb,&lcPlanner,  _1, _2);
  	server.setCallback(f);

	ros::Rate loop_rate(30);
    while(ros::ok()){

        ros::spinOnce();
        
        lcPlanner.plan();
        
        loop_rate.sleep();
    }
    return 0;
}
