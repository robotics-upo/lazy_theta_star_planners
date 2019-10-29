#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <upo_actions/MakePlanAction.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>
#include <std_srvs/Trigger.h>

class MissionInterface
{
    typedef actionlib::SimpleActionClient<upo_actions::MakePlanAction> MakePlanActionClient;

public:

    MissionInterface()
    {
        makePlanClient.reset(new MakePlanActionClient("Make_Plan", false));
        
        nh.reset(new ros::NodeHandle("~"));
        nh->param("mission_enabled", goals_by_file, (bool)1);
        nh->param("world_frame", world_frame, (std::string)"/map");

        if (goals_by_file)
        {
            ROS_DEBUG("Goal interface in Mission Mode");
            start_mission_server = nh->advertiseService("start_mission", &MissionInterface::startMission, this);
            reload_mission_data = nh->advertiseService("reload_mission_data", &MissionInterface::reloadMissionData, this);
            continue_mission_server = nh->advertiseService("continue_mission", &MissionInterface::continueMission, this);
            restore_mission_server = nh->advertiseService("restore_mission", &MissionInterface::restoreMission, this);

            //TODO Load Here the file
            missionLoaded = loadMissionData();
        }
        else
        {
            ROS_DEBUG("Goal interface in manual mode");
            goal_sub = nh->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &MissionInterface::goalCb, this);
        }
    }

    void processMissions()
    {
        if (goals_by_file)
        {
            if (!makePlanClient->isServerConnected())
            {
                ROS_WARN("Make Plan Server disconnected! :(, Waiting again for the server...");
                bool connected = makePlanClient->waitForServer(ros::Duration(2));
                if (connected)
                {
                    ROS_DEBUG("Make Plan Client and Server connected!");
                }
                else
                {
                    ROS_WARN("Timeout waiting for server...retrying");
                }

            }else if (!goals_queu.empty() && !goalRunning && doMission && goNext)//!Lots of flags :(
            {
                actionGoal.goal.global_goal = goals_queu.front();
                goals_queu.pop();
                makePlanClient->sendGoal(actionGoal.goal);
                goalRunning = true;
                goNext = false;
                ROS_DEBUG("Sending Goal: [%.2f, %.2f]\t[%.2f, %.2f, %.2f, %.2f]", actionGoal.goal.global_goal.pose.position.x, actionGoal.goal.global_goal.pose.position.y,
                         actionGoal.goal.global_goal.pose.orientation.x, actionGoal.goal.global_goal.pose.orientation.y,
                         actionGoal.goal.global_goal.pose.orientation.z, actionGoal.goal.global_goal.pose.orientation.w);
            }else if (goals_queu.empty() && doMission)
            {
                ROS_DEBUG_ONCE("No goals in the queu, Mission finished");
                goNext = true;
                doMission=false;
                goalRunning = false;
                goalReceived = false;
                missionLoaded = false;
            }
        }
        else if (goalReceived && !goalRunning)  //RViz goals case
        {
            makePlanClient->sendGoal(actionGoal.goal);
            goalRunning = true;
            goalReceived = false;
        }
        
        processState();
    }

private:

    void processState(){

        if (goalRunning)
        {
            if (makePlanClient->getState() == actionlib::SimpleClientGoalState::ABORTED && !lastGoalAborted)
            {
                //?It can mean the robot need an operator or something else
                ROS_DEBUG_ONCE("Goal aborted by the Global Planner");
                lastGoalAborted=true;
                ROS_DEBUG_ONCE("Holding on mission, waiting for manual intervention to clear the path");
                ROS_DEBUG_ONCE("Call service /mission_interface/restore_mission to continue to last goal");
            }

            if (makePlanClient->getState() == actionlib::SimpleClientGoalState::LOST)
            {
                //!Maybe resend goal?
                ROS_DEBUG("Goal Lost :'(");
            }
            if (makePlanClient->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
            {
                //!Do next goal?
                ROS_DEBUG("Goal Preempted");
                goalRunning = false;
            }
            if (makePlanClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_DEBUG("Goal succeded");
                goalRunning = false;
            }
        }
    }
    void goalCb(const geometry_msgs::PoseStampedConstPtr &goal)
    {

        ROS_DEBUG("Goal Interface: Sending action goal");
        actionGoal.goal.global_goal = *goal;
        actionGoal.goal_id.id += 1;
        actionGoal.goal_id.stamp = ros::Time::now();

        actionGoal.header.frame_id = world_frame;
        actionGoal.header.seq = rand();
        actionGoal.header.stamp = ros::Time::now();

        goalReceived = true;
        goalRunning = false;
    }
    bool restoreMission(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp){
        if(lastGoalAborted){
            makePlanClient->sendGoal(actionGoal.goal);
            goalRunning = true;
            goNext = false;
            lastGoalAborted=false;
        }
        return true;
    }
    bool startMission(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
    {
        if(doMission){
            resp.success = false;
            resp.message= "Mission is already started my friend, don't try to trick me ;)";
            return true;
        }
        if (!missionLoaded)
        {
            resp.success = false;
            resp.message = "Couldn't start Mission because file was not loaded";
        }
        else
        {
            resp.success = true;
            resp.message = "Starting Mission";
            doMission = true;
        }

        return true;
    }
    bool reloadMissionData(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
    {
        resp.success = loadMissionData();

        if (resp.success)
        {
            resp.message = "Mission Reloaded";
        }
        else
        {
            resp.message = "Couldn't reload mission";
        }

        return true;
    }
    bool continueMission(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
    {
        resp.success = false;
        if (!goals_queu.empty() && !goalRunning)
        { //If there are goals in the queu and no one is running it means the client is waiting to send another one
            resp.message = "Sending next goal...";
            resp.success = true;
            goNext = true;
        }
        if (goals_queu.empty())
        {
            resp.message = "Can't continue, no goals in the queu";
        }
        if (goalRunning)
        {
            resp.message = "The previous goal is still running, can't go to the next while previous goal running";
        }

        return true;
    }

    bool loadMissionData()
    {
        bool ret = true;
        std::string base_path = "mission/goal";
        geometry_msgs::PoseStamped goal;

        goal.header.frame_id = world_frame;
        
        int i = 1;

        while (nh->hasParam(base_path + std::to_string(i) + "/pose/x"))
        {
            if(i==1)
                for(size_t j = 1; goals_queu.size(); ++j )
                    goals_queu.pop();
            
            nh->param(base_path + std::to_string(i) + "/pose/x", goal.pose.position.x, (double)0);
            nh->param(base_path + std::to_string(i) + "/pose/y", goal.pose.position.y, (double)0);
            nh->param(base_path + std::to_string(i) + "/orientation/x", goal.pose.orientation.x, (double)0);
            nh->param(base_path + std::to_string(i) + "/orientation/y", goal.pose.orientation.y, (double)0);
            nh->param(base_path + std::to_string(i) + "/orientation/z", goal.pose.orientation.z, (double)0);
            nh->param(base_path + std::to_string(i) + "/orientation/w", goal.pose.orientation.w, (double)1);
            goal.header.seq = i;
            ROS_DEBUG("Goal %d (x,y)(x,y,z,w):\t[%.2f,%.2f]\t[%.2f,%.2f,%.2f,%.2f]", i, goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
            goals_queu.push(goal);

            ++i;
        }

        if (i == 1)
        {
            missionLoaded = false;
            ROS_DEBUG("Mission Not Loaded");
        }
        else
        {
            missionLoaded = true;
            ROS_DEBUG("Mission Loaded");
        }
        return missionLoaded;
    }

    ros::NodeHandlePtr nh; 
    ros::ServiceServer start_mission_server, reload_mission_data, continue_mission_server,restore_mission_server;
    ros::Subscriber goal_sub;

    upo_actions::MakePlanActionGoal actionGoal;

    std::queue<geometry_msgs::PoseStamped> goals_queu;
    std::unique_ptr<MakePlanActionClient> makePlanClient;
    std::string world_frame;

    //These flags are used by the mission mode
    bool missionLoaded = false;
    bool doMission = false;
    bool goNext=true;

    bool goalRunning = false;
    bool goalReceived = false; //This flag is only used when rviz goal mode is active
    bool goals_by_file;
    bool lastGoalAborted = false;
};

int main(int argc, char **argv)
{
    /* 
    *   This nodes listen to goals from topic and transform it to action message goal to request a plan
    *   It's useful to use when working with rviz for example if you want to test some features sending goals manually 
    */

    std::string node_name = "action_goal_interface_node";

    ros::init(argc, argv, node_name);

    MissionInterface iface;

    ros::Rate sleep_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        iface.processMissions();

        sleep_rate.sleep();
    }

    return 0;
}
