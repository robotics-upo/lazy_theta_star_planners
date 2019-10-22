/*
Local Planner using the ThetaStar Class 
Rafael Rey 2019 UPO
 */

#include <theta_star/LocalPlanner.hpp>

namespace PathPlanners
{

LocalPlanner::LocalPlanner(tf2_ros::Buffer *tfBuffer_)
{
    tfBuffer = tfBuffer_;
    configParams();
    configTopics();
    configTheta();
    configServices();
}
void LocalPlanner::configServices()
{
    
    //replanning_client_srv = nh_.serviceClient<std_srvs::Trigger>("/global_planner_node/global_replanning_service");
    costmap_clean_srv = nh_.serviceClient<std_srvs::Trigger>("/custom_costmap_node/reset_costmap");

    //TODO: Remove, the stop navigatin action should be infered from the executePath action state, arrived to goal also
    stop_nav_client_srv = nh_.serviceClient<std_srvs::Trigger>("/nav_node/pause_navigation_srv");

    stop_planning_srv = nh_.advertiseService("/local_planner_node/stop_planning_srv", &LocalPlanner::stopPlanningSrvCb, this);
    pause_planning_srv = nh_.advertiseService("/local_planner_node/pause_planning_srv", &LocalPlanner::pausePlanningSrvCb, this);

    arrived_to_goal_srv = nh_.advertiseService("/local_planner_node/arrived_to_goal", &LocalPlanner::arrivedToGoalSrvCb, this);

    execute_path_srv_ptr.reset(new ExecutePathServer(nh_, "Execute_Plan", false));
    execute_path_srv_ptr->registerGoalCallback(boost::bind(&LocalPlanner::executePathGoalServerCB,this));
    execute_path_srv_ptr->registerPreemptCallback(boost::bind(&LocalPlanner::executePathPreemptCB,this));
    execute_path_srv_ptr->start();


    navigate_client_ptr.reset(new NavigateClient("Navigation", true));


    
    //navigate_client_ptr->waitForServer();
}
void LocalPlanner::configParams()
{
    //Flags for flow control
    localCostMapReceived = false;
    globalTrajReceived = false;
    mapGeometryConfigured = false;
    doPlan = true;
    nh_.param("/local_planner_node/goal_weight", goal_weight, (float)1.5);
    nh_.param("/local_planner_node/cost_weight", cost_weight, (float)0.2);
    nh_.param("/local_planner_node/lof_distance", lof_distance, (float)1.5);
    nh_.param("/local_planner_node/occ_threshold", occ_threshold, (float)99);

    nh_.param("/local_planner_node/traj_dxy_max", traj_dxy_max, (float)1);
    nh_.param("/local_planner_node/traj_pos_tol", traj_pos_tol, (float)1);
    nh_.param("/local_planner_node/traj_yaw_tol", traj_yaw_tol, (float)0.1);

    nh_.param("/local_planner_node/world_frame", world_frame, (string) "/map");
    nh_.param("/local_planner_node/robot_base_frame", robot_base_frame, (string) "/base_link");
    nh_.param("/local_planner_node/local_costmap_infl_x", localCostMapInflationX, (float)1);
    nh_.param("/local_planner_node/local_costmap_infl_y", localCostMapInflationY, (float)1);
    nh_.param("/local_planner_node/border_space", border_space, (float)1);

    nh_.param("/local_planner_node/debug", debug, (bool)0);
    nh_.param("/local_planner_node/show_config", showConfig, (bool)0);

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Local Planner Node Configuration:\n");
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* with optim.: goal_weight = [%.2f]", goal_weight);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* modified: cost_weight = [%.2f], lof_distance = [%.2f]", cost_weight, lof_distance);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* occ_threshold = %f", occ_threshold);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Local Costmap Extra Borders = [%.2f, %.2f]", localCostMapInflationX, localCostMapInflationY);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Free space around local goal inside borders = [%.2f]", border_space);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Robot base frame: %s, World frame: %s", robot_base_frame.c_str(), world_frame.c_str());

    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "local_path";
    marker.id = rand();
    marker.type = RVizMarker::ARROW;
    marker.action = RVizMarker::ADD;
    marker.lifetime = ros::Duration(2);
    marker.scale.x = 0.4;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.pose.position.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    occ.data = false;
    is_running.data = true;
    impossible_calculate.data = false;
    startOk = false;

    impossibleCnt = 0;
    occGoalCnt = 0;
    number_of_points = 0;
    startIter = 1;
}
void LocalPlanner::dynRecCb(theta_star_2d::LocalPlannerConfig &config, uint32_t level)
{
    this->cost_weight = config.cost_weight;
    this->lof_distance = config.lof_distance;
    this->occ_threshold = config.occ_threshold;
    this->goal_weight = config.goal_weight;

    lcPlanner.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Dynamic reconfiguration required");
}
 void LocalPlanner::executePathPreemptCB()
  {
    ROS_INFO("Goal Preempted");
    // set the action state to preempted
    execute_path_srv_ptr->setPreempted();
  }
void LocalPlanner::executePathGoalServerCB()  // Note: "Action" is not appended to exe here
{
    ROS_INFO_COND(debug,"Local Planner Goal received in action server mode");
    upo_actions::ExecutePathGoalConstPtr path_shared_ptr; 
    path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();
    
    globalTrajReceived = true;
    globalTrajectory = path_shared_ptr->path;

    doPlan = true;
    startIter = 1;

    std_srvs::Trigger trg;
    costmap_clean_srv.call(trg);
    usleep(5e5);
    start_time = ros::Time::now();

    upo_actions::NavigateGoal nav_goal;
    size_t siz = globalTrajectory.points.size();
    nav_goal.global_goal.position.x = globalTrajectory.points.at(--siz).transforms[0].translation.x;
    nav_goal.global_goal.position.y =  globalTrajectory.points.at(--siz).transforms[0].translation.y;
    navigate_client_ptr->sendGoal(nav_goal);
                
}
void LocalPlanner::configTheta()
{
    string node_name = "local_planner_node";
    lcPlanner.initAuto(node_name, world_frame, goal_weight, cost_weight, lof_distance, &nh_);
    lcPlanner.setTimeOut(1);
    lcPlanner.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Theta Star Configured");
}
void LocalPlanner::configTopics()
{
    local_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/custom_costmap_node/costmap/costmap", 1, &LocalPlanner::localCostMapCb, this);

    //global_trj_sub = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/trajectory_tracker/input_trajectory", 1, &LocalPlanner::globalTrjCb, this);
    
    dist2goal_sub = nh_.subscribe<std_msgs::Float32>("/dist2goal", 1, &LocalPlanner::dist2GoalCb, this);
    
    trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/trajectory_tracker/local_input_trajectory", 0);
    vis_marker_traj_pub = nh_.advertise<visualization_msgs::MarkerArray>("/local_planner_node/visualization_marker_trajectory", 0);

    local_planning_time = nh_.advertise<std_msgs::Int32>("/local_planning_time", 0);

    if(debug)
        inf_costmap_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/local_costmap_inflated", 0);

    running_state_pub = nh_.advertise<std_msgs::Bool>("/local_planner_node/running", 0);
    occ_goal_pub = nh_.advertise<std_msgs::Bool>("/trajectory_tracker/local_goal_occupied", 0);
    impossible_to_find_sol_pub = nh_.advertise<std_msgs::Bool>("/trajectory_tracker/impossible_to_find", 0);
}

bool LocalPlanner::arrivedToGoalSrvCb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp){

    //If the path tracker call this service it means that the robot has arrived
    action_result.arrived = true;
    execute_path_srv_ptr->setSucceeded();    
    
}
void LocalPlanner::dist2GoalCb(const std_msgs::Float32ConstPtr &dist){
    d2goal = *dist;
}
bool LocalPlanner::stopPlanningSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep)
{

    globalTrajReceived = false;
    rep.message = "Waiting for a new global trajectory";
    rep.success = true;
    execute_path_srv_ptr->setAborted();
    return true;
}
bool LocalPlanner::pausePlanningSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep)
{

    rep.success = true;
    rep.message = "Planning resumed";

    if (doPlan){
        rep.message = "Planning paused";
    }else{
    }
    doPlan = !doPlan;
    
    return true;
}
//Calbacks and publication functions
void LocalPlanner::localCostMapCb(const nav_msgs::OccupancyGrid::ConstPtr &lcp)
{
    localCostMap = *lcp;
    localCostMapReceived = true;

    //ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Received local costmap");
    //First time the map is received, configure the geometric params
    if (!mapGeometryConfigured)
    {
        map_resolution = round(100 * (lcp->info.resolution)) / 100; //Because sometimes the map server shows not exact long numbers as 0.0500003212

        ws_x_max = lcp->info.width * map_resolution + 2 * localCostMapInflationX;
        ws_y_max = lcp->info.height * map_resolution + 2 * localCostMapInflationY;

        local_costmap_center.x = ws_x_max / 2;
        local_costmap_center.y = ws_y_max / 2;

        lcPlanner.loadMapParams(ws_x_max, ws_y_max, map_resolution);
        mapGeometryConfigured = true;

        ROS_INFO_COND(showConfig, PRINTF_GREEN "\t WorkSpace: X:[%.2f], Y:[%.2f]", ws_x_max, ws_y_max);
        ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Local Costmap Origin: [%.2f, %.2f]", local_costmap_center.x, local_costmap_center.y);
        ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Map: resol.= [%.2f]", map_resolution);
    }
}
//Global Input trajectory
//! Right now the subscirber is commented so it's not used.
//TODO: Remove in the future when the action lib communication it's implemented and tested
/*void LocalPlanner::globalTrjCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &traj)
{
    globalTrajectory = *traj;

    startIter = 1;
    globalTrajReceived = true;
    doPlan = true; //Restore the doPlan flag because we get a new trajectory do a new goal

    //Also clean the local costmap 
    std_srvs::Trigger trg;
    costmap_clean_srv.call(trg);
    usleep(5e5);

    ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Received global trajectory");
}
*/
void LocalPlanner::plan()
{

    running_state_pub.publish(is_running);
    number_of_points = 0; //Reset variable
    
    if(!execute_path_srv_ptr->isActive())
        return;
    
    ftime(&startT);
    if (globalTrajReceived && localCostMapReceived && doPlan && execute_path_srv_ptr->isActive())
    {
        if(navigate_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            action_result.arrived = true;
            execute_path_srv_ptr->setSucceeded();
            ROS_INFO_COND(debug,"Goal Succed");
        }   
        ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Global trj received and local costmap received");
        localCostMapReceived = false;

        if (!lcPlanner.setValidInitialPosition(local_costmap_center))
        {
            if (lcPlanner.searchInitialPosition2d(0.3))
            {
                startOk = true;
            }
            else
            {
                ROS_INFO("Local:Couldn't find a free point near start point, trying to clean the local costmap");
                std_srvs::Trigger trg;
                costmap_clean_srv.call(trg);
            }
        }
        else
        {
            startOk = true;
        } //To get sure the next time it calculate a path it has refreshed the local costmap
        if (startOk)
        {
            ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Calculating Local goal");
            localGoal = calculateLocalGoal();
            ROS_INFO_COND(debug, PRINTF_YELLOW "Local Goal: [%.2f, %.2f]", localGoal.x, localGoal.y);
            ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Adding borders to local costmap");
            inflateCostMap();
            ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Freeing space around local goal in the border");
            freeLocalGoal();
            
            if(debug)
                inf_costmap_pub.publish(localCostMapInflated); //Debug purposes

            lcPlanner.getMap(&localCostMapInflated);

            if (lcPlanner.setValidFinalPosition(localGoal))
            {
                
                ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Computing Local Path(2)");
                number_of_points = lcPlanner.computePath();
                ROS_INFO("Number of points: %d, occ: %d", number_of_points, occ.data);
                planningStatus.data="OK";
                if (occ.data)//If the local goal was previously occupied, reset flag and publish new status
                {
                    //ROS_INFO("Local: Local goal disocuupied");
                    occ.data = false;
                    occ_goal_pub.publish(occ);
                }
            }
            else
            {
                if (lcPlanner.searchFinalPosition2d(0.3))
                {
                    ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Computing Local Path after a free place next to goal found");
                    number_of_points = lcPlanner.computePath();
                    planningStatus.data="OK: Path Calculated after search around goal";
                    occGoalCnt = 0;
                }
                else if (occGoalCnt > 3)
                { //If it cant find a free position near local goal, it means that there is something there.
                    //Send message to tracker to stop
                    if(!occ.data){
                        occ.data = true;
                        occ_goal_pub.publish(occ);
                    }
                    
                    //And pause planning, under construction
                    doPlan = false;
                    ROS_INFO_COND(debug, PRINTF_YELLOW "Pausing planning, final position busy");
                    planningStatus.data = "Final position Busy, Cancelling goal";
                    //TODO What to tell to the path tracker
                    navigate_client_ptr->cancelGoal();
                    //In order to resume planning, someone must call the pause/resume planning Service that will change the flag to true
                }
                else
                {
                    occGoalCnt++;
                }
            }

            if (number_of_points > 0 && !occ.data)
            {
                ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Building and publishing local Path");
                buildAndPubTrayectory();
                ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Local Path build and published");
                publishTrajMarker();
                ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Published trajectory markers");
                startOk = false;

               
                if (impossibleCnt > 0) //If previously the local planner couldn t find solution, reset
                {
                    //ROS_INFO("Local: Impossible reseted");
                    impossible_calculate.data = false;
                    impossible_to_find_sol_pub.publish(impossible_calculate);
                    impossibleCnt = 0;

                    //TODO: Change this to the action command
                    std_srvs::Trigger trg;
                    stop_nav_client_srv.call(trg);
                }
            }
            else if (!occ.data)
            {
                impossibleCnt++;
                //ROS_INFO("Local: +1 impossible");
                if (impossibleCnt > 2)
                {

                    impossible_calculate.data = true;
                    impossible_to_find_sol_pub.publish(impossible_calculate);
                    navigate_client_ptr->cancelGoal();
                    planningStatus.data="Requesting new global path, navigation cancelled";
                    ROS_WARN("Requesting new global path to same global goal, %d", impossibleCnt);
                    //TODO These triggers will be removed
                    std_srvs::Trigger srv,stop_nav;

                    //replanning_client_srv.call(srv);
                    
                    execute_path_srv_ptr->setAborted();
                    globalTrajReceived = false;
                    //execute_path_srv_ptr->
                    //TODO: Change this to the action command
                    if(impossibleCnt == 3){
                        stop_nav_client_srv.call(stop_nav);
                        
                    }

                    //TODO 
                    //if(execute_path_srv_ptr->isNewGoalAvailable())

                    //TODO This is the global planner responding, think about how to get new status
                    if (srv.response.success)
                    {
                        impossibleCnt = 0;
                    }
                    else
                    {
                        ROS_WARN("Local Planner: Failed to obstain new global path from global planner");
                    }
                    startOk = false;
                }
            }
        }
    }
    ftime(&finishT);

    seconds = finishT.time - startT.time - 1;
    milliseconds = (1000 - startT.millitm) + finishT.millitm;
    time_spent_msg.data = (milliseconds + seconds * 1000);
    publishExecutePathFeedback();
    //if (debug)
    //    showTime(PRINTF_YELLOW "Time spent", startT, finishT);

    local_planning_time.publish(time_spent_msg);
}
void LocalPlanner::publishExecutePathFeedback(){
    // planningRate;
    //waypointGoingTo;
    // planningStatus;
    if(planningStatus.data.find("OK") > 0){
        planningRate.data = 1000/milliseconds;
    }else{
        planningRate.data = 0;
    }
    
    exec_path_fb.global_waypoint = waypointGoingTo;
    exec_path_fb.planning_rate = planningRate;
    exec_path_fb.status = planningStatus;
    execute_path_srv_ptr->publishFeedback(exec_path_fb);

}
geometry_msgs::Vector3 LocalPlanner::calculateLocalGoal()
{

    geometry_msgs::Vector3Stamped A, B, C;

    trajectory_msgs::MultiDOFJointTrajectory globalTrajBLFrame = globalTrajectory;
    //First we transform the trajectory published by global planner to base_link frame

    geometry_msgs::TransformStamped tr = getTfMapToRobot();

    globalTrajBLFrame.header.frame_id = robot_base_frame;

    for (size_t i = 0; i < globalTrajectory.points.size(); i++)
    {
        globalTrajBLFrame.points[i].transforms[0].translation.x -= tr.transform.translation.x;
        globalTrajBLFrame.points[i].transforms[0].translation.y -= tr.transform.translation.y;
    }

    //Ya esta referida al base_link. Ahora la recorro desde i=1(porque i=0 es siempre la pos del base_link que al pasarla al sistema base_link sera (0,0))

    for (size_t i = startIter; i < globalTrajectory.points.size(); i++, startIter++)
    {
        B.vector.x = globalTrajBLFrame.points[i].transforms[0].translation.x;
        B.vector.y = globalTrajBLFrame.points[i].transforms[0].translation.y;

        C.vector.x = B.vector.x + (ws_x_max) / 2;
        C.vector.y = B.vector.y + (ws_y_max) / 2;

        if (fabs(B.vector.x) > (ws_x_max / 2 - localCostMapInflationX) || fabs(B.vector.y) > (ws_y_max / 2 - localCostMapInflationY) || (i == globalTrajectory.points.size() - (size_t)1))
        {
            A.vector.x = globalTrajBLFrame.points[i - 1].transforms[0].translation.x;
            A.vector.y = globalTrajBLFrame.points[i - 1].transforms[0].translation.y;
            if (fabs(B.vector.x) > ws_x_max / 2 || fabs(B.vector.y) > ws_y_max / 2)
            {

                C.vector.x = A.vector.x + (ws_x_max) / 2;
                C.vector.y = A.vector.y + (ws_y_max) / 2;

                while (C.vector.x < (ws_x_max - localCostMapInflationX + map_resolution) && C.vector.x > localCostMapInflationX - 2 * map_resolution &&
                       C.vector.y < (ws_y_max - localCostMapInflationY + map_resolution) && C.vector.y > localCostMapInflationY - 2 * map_resolution)
                { //Put the point between i-1 and i
                    C.vector.x += map_resolution * (B.vector.x - A.vector.x) / sqrtf(pow(B.vector.x - A.vector.x, 2) + pow(B.vector.y - A.vector.y, 2));
                    C.vector.y += map_resolution * (B.vector.y - A.vector.y) / sqrtf(pow(B.vector.x - A.vector.x, 2) + pow(B.vector.y - A.vector.y, 2));
                }
                startIter = i - 1;
            }

            break;
        }
    }
    //TODO: Mejorar esta chapuza
    while (C.vector.x > (ws_x_max - map_resolution) || C.vector.y > (ws_y_max - map_resolution))
    {
        C.vector.x -= map_resolution * (B.vector.x - A.vector.x) / sqrtf(pow(B.vector.x - A.vector.x, 2) + pow(B.vector.y - A.vector.y, 2));
        C.vector.y -= map_resolution * (B.vector.y - A.vector.y) / sqrtf(pow(B.vector.x - A.vector.x, 2) + pow(B.vector.y - A.vector.y, 2));
    }
    if (C.vector.x == 0)
        C.vector.x += 2 * map_resolution;
    if (C.vector.y == 0)
        C.vector.y += 2 * map_resolution;

    C.vector.x = floor(C.vector.x * 10 + 10 * map_resolution) / 10;

    C.vector.y = floor(C.vector.y * 10 + 10 * map_resolution) / 10;

    return C.vector;
}

geometry_msgs::TransformStamped LocalPlanner::getTfMapToRobot()
{
    geometry_msgs::TransformStamped ret;

    try
    {
        ret = tfBuffer->lookupTransform(world_frame, robot_base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return ret;
}

void LocalPlanner::publishTrajMarker()
{
    markerTraj.markers.clear();

    marker.action = RVizMarker::DELETEALL;

    markerTraj.markers.push_back(marker);
    vis_marker_traj_pub.publish(markerTraj);
    marker.action = RVizMarker::ADD;

    markerTraj.markers.clear();
    marker.header.stamp = ros::Time::now();

    for (size_t i = 0; i < localTrajectory.points.size(); i++)
    {
        marker.pose.position.x = localTrajectory.points[i].transforms[0].translation.x;
        marker.pose.position.y = localTrajectory.points[i].transforms[0].translation.y;
        marker.pose.orientation = localTrajectory.points[i].transforms[0].rotation;
        marker.id = rand();
        markerTraj.markers.push_back(marker);
    }
    vis_marker_traj_pub.publish(markerTraj);
}

void LocalPlanner::buildAndPubTrayectory()
{
    trajectory_msgs::MultiDOFJointTrajectoryPoint goal_temp;
    geometry_msgs::Transform temp1;
    //La trayectoria se obtiene en el frame del local costmap, que tiene la misma orientacion que el map pero esta centrado en el base_link

    localTrajectory.points.clear();

    lcPlanner.getTrajectoryYawInAdvance(localTrajectory, getTfMapToRobot().transform);

    for (size_t i = 0; i < localTrajectory.points.size(); i++)
    {
        localTrajectory.points[i].transforms[0].translation.x += localCostMapInflated.info.origin.position.x;
        localTrajectory.points[i].transforms[0].translation.y += localCostMapInflated.info.origin.position.y;
    }
    localGoal.x += localCostMapInflated.info.origin.position.x;
    localGoal.y += localCostMapInflated.info.origin.position.y;

    temp1.translation.x = localGoal.x;
    temp1.translation.y = localGoal.y;

    temp1.rotation = globalTrajectory.points[startIter].transforms[0].rotation;

    goal_temp.transforms.resize(1, temp1);

    localTrajectory.points.push_back(goal_temp);

    localTrajectory.header.stamp = ros::Time::now();

    trajectory_pub.publish(localTrajectory);
    //ROS_WARN_THROTTLE(1, "Average dist 2 obstacles: %.2f", lcPlanner.getAvDist2Obs());
}
//Auxiliar functions
void LocalPlanner::showTime(string message, struct timeb st, struct timeb ft)
{
    float seconds, milliseconds;

    seconds = ft.time - st.time - 1;

    milliseconds = (1000 - st.millitm) + ft.millitm;
    cout << message << (milliseconds + seconds * 1000) << " ms" << endl;
}
//Add the occupied borders around received costmap
void LocalPlanner::inflateCostMap()
{
    //Free the data field and populate the header and info with the corresponding information
    localCostMapInflated.data.resize(0);

    localCostMapInflated.header.frame_id = localCostMap.header.frame_id;
    localCostMapInflated.header.seq = localCostMap.header.seq;
    localCostMapInflated.header.stamp = ros::Time(0);

    localCostMapInflated.info.height = localCostMap.info.height + 2 * localCostMapInflationY / map_resolution;
    localCostMapInflated.info.width = localCostMap.info.width + 2 * localCostMapInflationX / map_resolution;

    localCostMapInflated.info.resolution = map_resolution;

    localCostMapInflated.info.origin.position.x = localCostMap.info.origin.position.x - localCostMapInflationX;
    localCostMapInflated.info.origin.position.y = localCostMap.info.origin.position.y - localCostMapInflationY;

    float l = localCostMapInflated.info.width * localCostMapInflationY / map_resolution;
    //iterator to know where we are
    int iter = 0;
    //Upper border
    for (int i = 0; i < l; i++)
    {
        localCostMapInflated.data.push_back(100);
    } //Costmap from left border to right border, from upper original limit to lower original limit
    for (int i = 0; i < localCostMap.info.height; i++)
    {
        for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
            localCostMapInflated.data.push_back(100);

        for (int k = 0; k < localCostMap.info.width; k++)
        {
            localCostMapInflated.data.push_back(localCostMap.data[iter]);
            iter++;
        }
        for (int l = 0; l < localCostMapInflationX / map_resolution; l++)
            localCostMapInflated.data.push_back(100);
    }
    //Lower boder
    for (int i = 0; i < l; i++)
        localCostMapInflated.data.push_back(100);
}
//Set to 0 the positions near the local goal in the border
void LocalPlanner::freeLocalGoal()
{
    //localGoal is a Vector3
    //This function free space around localGoal in the local costmap border

    /**
     * En esta primera version se consideran los 8 subcasos a pelo
     * * Sea w la anchura sin el borde inflado y h la altura sin el borde inflado
     * * Sea W la anchura con el borde inflado y H la altura con el borde inflado (W = w+2*infladoHorizontal, H = h+2*infladoVertical)
     * ! _ _ _ _ __ _ _ _ _
     * ! |                 |
     * ! | 1      2      3 |
     * ! |  |-----------|  |
     * ! |  |           |  |
     * ! |  |           |  |
     * ! | 8|           |4 |
     * ! |  |           |  |
     * ! |  |           |  |
     * ! |  |-----------|  |
     * ! |7       6      5 |
     * ! |_ _ _ _ _ _ _ _ _|
     * 
     * El dibujito deberia estar centrado y ser simetrico
     * ?1,3,5 y 7: Esquinas: En estos casos se libera espeacio horizontal y vertical
     * ?2,4,6 y 8: Bordes laterales: En estos casos liberamos la franja vertical/horizontal correspondiente segun sean los casos 8,4 o 2,6
     * 
    **/

    //First we do is to detect in which case we are
    //Primero: Esquinas: modulo de las componentes x e y mayor que el w y h
    //Luego si no es una esquina
    // TODO: Lo mas eficiente no es esto, sino comprobar primero los casos mas probables, que serian las 4 paredes, y luego las 4 esquinas)

    //Para todos los bucles siempre es igual
    // ! i: Numero de fila
    // ! j: columna
    int st, end;
    // ROS_INFO_COND(debug, PRINTF_GREEN "1");
    if (localGoal.y > localCostMapInflated.info.height * map_resolution - localCostMapInflationY)
    {
        //Esquina 1 o 3 o borde 2
        if (localGoal.x < localCostMapInflationX) //1
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "2");
            for (int i = localCostMapInflated.info.height - 2 * localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i++)
                for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "3");
            for (int i = localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height; i++)
                for (int j = 0; j < 2 * localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else if (localGoal.x > localCostMapInflated.info.width * map_resolution - localCostMapInflationX) //5
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "4");
            for (int i = localCostMapInflated.info.height - 2 * localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i++)
                for (int j = localCostMapInflated.info.width - localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "5");
            for (int i = localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height; i++)
                for (int j = localCostMapInflated.info.width - 2 * localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else //Borde 2
        {

            st = (localGoal.x - border_space) / map_resolution;
            if (st < 0)
                st = 0;

            end = (localGoal.x + border_space) / map_resolution;
            if (end > ws_x_max / map_resolution)
                end = ws_x_max / map_resolution;

            // ROS_INFO_COND(debug, PRINTF_GREEN "6");
            for (int i = localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height; i++)
                for (int j = st; j < end; j++)
                    localCostMapInflated.data[i * localCostMapInflated.info.width + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "7");
        }
    }
    else if (localGoal.y < localCostMapInflationY)
    {
        // ROS_INFO_COND(debug, PRINTF_GREEN "8");                                                      //Esquina 3 o 1 o borde 6
        if (localGoal.x > localCostMapInflated.info.width * map_resolution - localCostMapInflationX) //3
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "9");
            for (int i = 0; i < localCostMapInflationY / map_resolution; i++)
                for (int j = localCostMapInflated.info.width - 2 * localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "10");
            for (int i = localCostMapInflationY / map_resolution; i < 2 * localCostMapInflationY / map_resolution; i++)
                for (int j = localCostMapInflated.info.width - localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "11");
        }
        else if (localGoal.x < localCostMapInflationX) //1
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "12");
            for (int i = 0; i < localCostMapInflationY / map_resolution; i++) //Filas
                for (int j = 0; j < 2 * localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "13");
            for (int i = localCostMapInflationY / map_resolution; i < 2 * localCostMapInflationY / map_resolution; i++)
                for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else //Borde 6
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "14");

            st = (localGoal.x - border_space) / map_resolution;
            if (st < 0)
                st = 0;

            end = (localGoal.x + border_space) / map_resolution;
            if (end > ws_x_max / map_resolution)
                end = ws_x_max / map_resolution;

            for (int i = 0; i < localCostMapInflationY / map_resolution; i++)
                for (int j = st; j < end; j++)
                    localCostMapInflated.data[i * localCostMapInflated.info.width + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "15");
        }
    } //Si hemos llegado hasta aqui sabemos que la y esta dentro del costmap original
    else if (localGoal.x < localCostMapInflationX)
    { //Borde 8
        //Se recorren las filas desde la primera hasta la ultima y se ponen a 0 los N primeros valores de cada fila(numero de columnas infladas)
        // ROS_INFO_COND(debug, PRINTF_GREEN "16");

        st = (localGoal.y - border_space) / map_resolution;
        if (st < 0)
            st = 0;

        end = (localGoal.y + border_space) / map_resolution;
        if (end > ws_y_max / map_resolution)
            end = ws_y_max / map_resolution;

        for (int i = st; i < end; i++)
            for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
                localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        // ROS_INFO_COND(debug, PRINTF_GREEN "17");
    }
    else if (localGoal.x > localCostMapInflated.info.width * map_resolution - localCostMapInflationX)
    { //Borde 4
        //Se recorren las filas desde la primera hasta la ultima y se ponen a 0 los N ultimos valores de cada fila (numero de columnas infladas)
        // ROS_INFO_COND(debug, PRINTF_GREEN "18");
        st = (localGoal.y - border_space) / map_resolution;
        end = (localGoal.y + border_space) / map_resolution;
        if (st < 0)
            st = 0;
        if (end > ws_y_max / map_resolution)
            end = ws_y_max / map_resolution;

        for (int i = st; i < end; i++)
            for (int j = localCostMapInflated.info.width - localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;

        // ROS_INFO_COND(debug, PRINTF_GREEN "19");
    }
    else
    {
        ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: No need to free space in the border, local goal inside original local costmap");
    }
}

} // namespace PathPlanners