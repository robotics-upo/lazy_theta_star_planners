#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <theta_star_2d/GoalCmd.h>


theta_star_2d::GoalCmd goal_cmd;
std::unique_ptr<ros::ServiceClient> srv_client_ptr;
bool goalRec;
void goalCb(const geometry_msgs::PoseStampedConstPtr &goal){
    goal_cmd.request.pose.pose = goal->pose;
    goalRec = true;
}
int main(int argc, char **argv)
{
    /* 
    *   This nodes listen to goals from topic and transform it to action message goal to request a plan
    *   It's useful to use when working with rviz for example if you want to test some features sending goals manually 
    */

	std::string node_name = "action_goal_interface_node";
	ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    goalRec = false;
    ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1,goalCb);
    ros::ServiceClient goal_srv = n.serviceClient<theta_star_2d::GoalCmd>("/global_planner_node/plan");
    
    goal_srv.waitForExistence();
    

    ros::Rate sleep_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        if(goalRec){
            goal_srv.call(goal_cmd);
            goalRec = false;
        }
        sleep_rate.sleep();
    }
    

    return 0;
}
