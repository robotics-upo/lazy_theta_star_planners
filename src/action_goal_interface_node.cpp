#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <upo_actions/MakePlanAction.h>
#include <actionlib/client/simple_action_client.h>

upo_actions::MakePlanActionGoal actionGoal;

bool goalRec = false;
bool goalRunning = false;
int n = 0;

void goalCb(const geometry_msgs::PoseStampedConstPtr &goal)
{
    ROS_INFO("Goal Interface: Sending action goal");
    actionGoal.goal.global_goal = *goal;
    actionGoal.goal_id.id = n++;
    actionGoal.goal_id.stamp = ros::Time::now();

    actionGoal.header.frame_id = "map";
    actionGoal.header.seq = rand();
    actionGoal.header.stamp = ros::Time::now();

    goalRec = true;
    goalRunning = true;
    
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

    actionlib::SimpleActionClient<upo_actions::MakePlanAction> makePlanClient("Make_Plan", false);

    ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1, goalCb);
    
    ros::Rate sleep_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        if (goalRec)
        {
            makePlanClient.sendGoal(actionGoal.goal);
            goalRec = false;
        }

        
        if (goalRunning)
        {
            ROS_INFO_THROTTLE(1, "Status: %s", makePlanClient.getState().toString().c_str());
            if (makePlanClient.getState().isDone()){
                goalRunning = false;
                ROS_INFO("Done! :)");
            }
                
        }
        //if (!makePlanClient.isServerConnected())
        //{
        //    ROS_INFO("Server disconnected! :(, Waiting again for the server...");
        //    makePlanClient.waitForServer();
        //}

        sleep_rate.sleep();
    }

    return 0;
}
