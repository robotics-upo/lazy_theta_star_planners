#include <theta_star/GlobalPlanner.hpp>


using namespace PathPlanners;


int main(int argc, char **argv)
{

	string node_name = "global_planner_node";
	ros::init(argc, argv, node_name);

    GlobalPlanner globalPlanner(node_name);

    dynamic_reconfigure::Server<theta_star_2d::GlobalPlannerConfig> server;
  	dynamic_reconfigure::Server<theta_star_2d::GlobalPlannerConfig>::CallbackType f;

  	f = boost::bind(&GlobalPlanner::dynReconfCb,&globalPlanner,  _1, _2);
  	server.setCallback(f);

	ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
        globalPlanner.plan();

        loop_rate.sleep();
    }
    return 0;
}
