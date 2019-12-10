#include <ros/ros.h>
#include <upo_actions/ExecuteMissionAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<upo_actions::ExecuteMissionAction> ExecuteMissionActionClient;

int main(int argc, char **argv)
{
    ros::init(argc,argv, "hmi_fake");
    ros::NodeHandle nh("~");
    ExecuteMissionActionClient hmi_client(nh, "/Execute_Mission",true);
    upo_actions::ExecuteMissionGoal mission_goal;

    bool connected = false;
    
    while(!connected){
        connected =hmi_client.waitForServer(ros::Duration(2));
        ROS_WARN("Retrying connection");
    }
    
    if(argc==2){
        std::string mis = argv[1];
        mission_goal.mission_name.data =std::string(argv[1]);
        hmi_client.sendGoal(mission_goal);
        ROS_INFO("Goal sended");
    }else{
        ROS_ERROR("BAD USAGE: rosrun theta_star_2d hmi_fake path_to_mission_file.yaml");
        std::exit(0);
    }
    
    ros::spin();
    return 0;
}
