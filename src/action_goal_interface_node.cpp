#include <ros/ros.h>
#include <string>
#include <fstream>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <upo_actions/MakePlanAction.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>
#include <std_srvs/Trigger.h>
#include <ros/package.h>
#include <theta_star_2d/SaveMission.h>
#include <ros/console.h>

class MissionInterface
{
    typedef actionlib::SimpleActionClient<upo_actions::MakePlanAction> MakePlanActionClient;

public:
    MissionInterface()
    {
        makePlanClient.reset(new MakePlanActionClient("Make_Plan", false));
        nh.reset(new ros::NodeHandle("~"));
        nh->param("mission_enabled", goals_by_file, (bool)true);
        nh->param("world_frame", world_frame, (std::string) "map");
        nh->param("override_in_rviz_mode", override, (bool)true);
        nh->param("hmi_ns", hmi_ns, (std::string)"HMI");       
        //This topic will add the current last goal succedeed(i.e. the current robot pose) to the goals queue just before the shelter
        goal_red_marker_sub = nh->subscribe<geometry_msgs::PoseStamped>("add_red_waypoint", 1, &MissionInterface::redPointCb,this);
        build_mission_points = nh->subscribe<geometry_msgs::PoseStamped>("add_waypoint", 1, &MissionInterface::addWaypointCb,this);
        save_mission_server = nh->advertiseService("save_mission", &MissionInterface::saveMissionSrvCB, this);

        if (goals_by_file)
        {
            ROS_DEBUG("Goal interface in Mission Mode");
            start_mission_server = nh->advertiseService("start_mission", &MissionInterface::startMission, this);
            reload_mission_data = nh->advertiseService("reload_mission_data", &MissionInterface::reloadMissionData, this);
            continue_mission_server = nh->advertiseService("continue_mission", &MissionInterface::continueMission, this);
            restore_mission_server = nh->advertiseService("restore_mission", &MissionInterface::restoreMission, this);

            missionLoaded = loadMissionData();
        }
        else
        {
            ROS_DEBUG("Goal interface in manual mode");
            rviz_goal_sub = nh->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &MissionInterface::goalCb, this);
        }
    }

    void processMissions()
    {
        
        if (goals_by_file)
        {
            if (!makePlanClient->isServerConnected())
            {
                bool connected = makePlanClient->waitForServer(ros::Duration(2));
                if (connected)
                {
                    ROS_DEBUG("Make Plan Client and Server connected!");
                }
                else
                {
                    ROS_DEBUG("Timeout waiting for server...retrying");
                }
            }
            else if (!goals_queu.empty() && doMission && goNext)
            {
                actionGoal.goal.global_goal = goals_queu.front();
                goals_queu.pop();
                makePlanClient->sendGoal(actionGoal.goal);
                goalRunning = true;
                goNext = false;
                ROS_INFO_NAMED(hmi_ns,"Sending Robot to next inspection point: [%.2f, %.2f]\t[%.2f, %.2f, %.2f, %.2f]", actionGoal.goal.global_goal.pose.position.x, actionGoal.goal.global_goal.pose.position.y,
                          actionGoal.goal.global_goal.pose.orientation.x, actionGoal.goal.global_goal.pose.orientation.y,
                          actionGoal.goal.global_goal.pose.orientation.z, actionGoal.goal.global_goal.pose.orientation.w);
            }
            else if (goals_queu.empty() && doMission && !isGoalActive())
            {
                ROS_INFO_NAMED(hmi_ns,"Robot arrived to shelter, mission finished");
                goNext = true;
                doMission = false;
                goalRunning = false;
                goalReceived = false;
                missionLoaded = false;
            }
        }
        else if (goalReceived && (override || !isGoalActive()) )
        {
            makePlanClient->sendGoal(actionGoal.goal);
            goalRunning = true;
            goalReceived = false;
        }
        

        processState();
    }

private:
   
    bool isGoalActive()
    {
        if (makePlanClient->getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    void processState()
    {

        if (goalRunning)
        {
            if (makePlanClient->getState() == actionlib::SimpleClientGoalState::ABORTED && !lastGoalAborted)
            {
                //?It can mean the robot need an operator or something else
                ROS_DEBUG_ONCE("Goal aborted by the Global Planner");
                lastGoalAborted = true;
                ROS_DEBUG_ONCE("Holding on mission, waiting for manual intervention to clear the path");
                ROS_DEBUG_ONCE("Call service /mission_interface/restore_mission to continue to last goal");
            }

            if (makePlanClient->getState() == actionlib::SimpleClientGoalState::LOST)
            {
                //!Maybe resend goal?
                ROS_INFO_NAMED(hmi_ns,"Communication fail, goal lost");
            }
            if (makePlanClient->getState() == actionlib::SimpleClientGoalState::REJECTED)
            {
                ROS_DEBUG("Goal Rejected :'(");
            }
            if (makePlanClient->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
            {
                //!Do next goal?
                ROS_DEBUG("Goal Preempted");
                goalRunning = false;
            }
            if (makePlanClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO_NAMED(hmi_ns,"Robot arrived to inspection point");
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
    bool restoreMission(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
    {
        if (lastGoalAborted)
        {
            makePlanClient->sendGoal(actionGoal.goal);
            goalRunning = true;
            goNext = false;
            lastGoalAborted = false;
        }
        return true;
    }
    bool startMission(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
    {
        if (doMission)
        {
            resp.success = false;
            resp.message = "Mission is already started";
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
            ROS_INFO_NAMED(hmi_ns,"Mission inspection points loaded");
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
            resp.success = false;
        }
        if (goalRunning)
        {
            resp.message = "The previous goal is still running, can't go to the next while previous goal running";
            resp.success = false;
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

        //First get shelter position
        nh->param("shelter/pose/x", shelter_position.pose.position.x, (double)0);
        nh->param("shelter/pose/y", shelter_position.pose.position.y, (double)0);
        nh->param("shelter/orientation/x", shelter_position.pose.orientation.x, (double)0);
        nh->param("shelter/orientation/y", shelter_position.pose.orientation.y, (double)0);
        nh->param("shelter/orientation/z", shelter_position.pose.orientation.z, (double)0);
        nh->param("shelter/orientation/w", shelter_position.pose.orientation.w, (double)1);

        while (nh->hasParam(base_path + std::to_string(i) + "/pose/x"))
        {
            if (i == 1)
                for (size_t j = 1; goals_queu.size(); ++j)
                    goals_queu.pop();

            nh->param(base_path + std::to_string(i) + "/pose/x", goal.pose.position.x, (double)0);
            nh->param(base_path + std::to_string(i) + "/pose/y", goal.pose.position.y, (double)0);
            nh->param(base_path + std::to_string(i) + "/orientation/x", goal.pose.orientation.x, (double)0);
            nh->param(base_path + std::to_string(i) + "/orientation/y", goal.pose.orientation.y, (double)0);
            nh->param(base_path + std::to_string(i) + "/orientation/z", goal.pose.orientation.z, (double)0);
            nh->param(base_path + std::to_string(i) + "/orientation/w", goal.pose.orientation.w, (double)1);
            goal.header.seq = i;
            ROS_INFO_NAMED(hmi_ns,"Goal %d (x,y)(x,y,z,w):\t[%.2f,%.2f]\t[%.2f,%.2f,%.2f,%.2f]", i, goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
            goals_queu.push(goal);

            ++i;
        }
        goals_queu.push(shelter_position);
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
    /*
    *   This callback will receive the list of the points sent by the HMI 
    */       
    void addWaypointCb(const geometry_msgs::PoseStampedConstPtr &p){

        mission.push_back(*p);
    }
    void redPointCb(const geometry_msgs::PoseStampedConstPtr &pose){

        ROS_INFO_NAMED(hmi_ns,"Found red marker at current inspection point. Adding point for ulterior inspection");
        if(doMission && goals_queu.size() > 1){
            std::queue<geometry_msgs::PoseStamped> goals_queu_temp;
            geometry_msgs::PoseStamped temp_pose;
            size_t q_s=goals_queu.size();

            for(size_t i=1; i< q_s; ++i){
                temp_pose = goals_queu.front();
                goals_queu_temp.push(temp_pose);
                goals_queu.pop();
            }
            goals_queu_temp.push(*pose);   
            temp_pose = goals_queu.front();
            goals_queu_temp.push(temp_pose);
            goals_queu.pop();
            goals_queu = goals_queu_temp;
        }
    }
    bool saveMissionSrvCB(theta_star_2d::SaveMissionRequest &req, theta_star_2d::SaveMissionResponse &rep){

        if(!doMission){
            writeMission(req.mission_name);
            rep.success=true;
            rep.message="Mission succesfully wrote to file";
        }else{
            rep.success=false;
            rep.message="You can't save the mission while doing one";
        }
        
        return true;
    }
    void writeMission(std::string file_name){
        
        //It means the HMI stopped sending goals so we only need to process the mission pose stamped vector
        std::string yaml_path =ros::package::getPath("theta_star_2d")+"/cfg/missions/"+file_name+".yaml";
        std::ofstream mission_f;

        mission_f.open(yaml_path, std::ios_base::out | std::ios_base::trunc);
        mission_f<<"mission:"<<std::endl;
        
        int count=0;
        
        for(auto it:mission){
            mission_f<<"  goal"<<++count<<":"<<std::endl;
            mission_f<<"    pose:"<<std::endl;
            mission_f<<"      x: "<<it.pose.position.x<<std::endl;
            mission_f<<"      y: "<<it.pose.position.y<<std::endl;
            mission_f<<"  orientation:"<<std::endl;
            mission_f<<"      x: "<<it.pose.orientation.x<<std::endl;
            mission_f<<"      y: "<<it.pose.orientation.y<<std::endl;
            mission_f<<"      z: "<<it.pose.orientation.z<<std::endl;
            mission_f<<"      w: "<<it.pose.orientation.w<<std::endl;
        }
        mission_f.close();
        
        mission.clear();
        
        system("rosparam delete /mission_interface/mission");
        std::string cmd="rosparam load "+yaml_path+" "+nh->getNamespace();
        system(cmd.c_str());
        loadMissionData();
        ROS_INFO_NAMED(hmi_ns,"Mission writed to file and loaded. Ready to start it");
        
    }
    //!HMI interac 
    geometry_msgs::PoseStamped temp_pose, shelter_position;
    std::vector<geometry_msgs::PoseStamped> mission;
    std::string hmi_ns;
    //!HMI interac

    ros::NodeHandlePtr nh;
    ros::ServiceServer start_mission_server, reload_mission_data, continue_mission_server, restore_mission_server;
    ros::ServiceServer save_mission_server;
    ros::Subscriber rviz_goal_sub, goal_hmi_sub,goal_red_marker_sub,build_mission_points;

    upo_actions::MakePlanActionGoal actionGoal;

    std::queue<geometry_msgs::PoseStamped> goals_queu;
    std::unique_ptr<MakePlanActionClient> makePlanClient;
    std::string world_frame;

    //These flags are used by the mission mode
    bool missionLoaded = false;
    bool doMission = false;
    bool goNext = true;

    bool goalRunning = false;
    bool goalReceived = false; //This flag is only used when rviz goal mode is active
    bool goals_by_file;
    bool lastGoalAborted = false;

    bool override;
};

int main(int argc, char **argv)
{
    /* 
    *   This nodes listen to goals from topic and transform it to action message goal to request a plan
    *   It's useful to use when working with rviz for example if you want to test some features sending goals manually 
    */

    std::string node_name = "mission_interface";

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
