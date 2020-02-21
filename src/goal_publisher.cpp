#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "goal_publisher_node");
    ros::NodeHandle lnh("~");
    geometry_msgs::PoseStamped goal;

    ros::Publisher goal_pub = lnh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    std::string base_path;
    if (argc > 2)
    {
        base_path = argv[1];
        if(!lnh.getParam(base_path+"/pose/x",goal.pose.position.x)){
            ROS_ERROR("Goal not found in param server");
            exit(0);
        }

        lnh.param(base_path + "/pose/x", goal.pose.position.x, (double)0);
        lnh.param(base_path + "/pose/y", goal.pose.position.y, (double)0);
        lnh.param(base_path + "/pose/z", goal.pose.position.z, (double)0);
        lnh.param(base_path + "/orientation/x", goal.pose.orientation.x, (double)0);
        lnh.param(base_path + "/orientation/y", goal.pose.orientation.y, (double)0);
        lnh.param(base_path + "/orientation/z", goal.pose.orientation.z, (double)0);
        lnh.param(base_path + "/orientation/w", goal.pose.orientation.w, (double)1);
        goal_pub.publish(goal);
    }
    else
    {
        ROS_ERROR("Bad syntax, example: rosrun theta_star_2d goal_publisher_node wall_point");
    }

    return 0;
}