/*

*/

#include <theta_star/GlobalPlanner.hpp>

namespace PathPlanners
{
GlobalPlanner::GlobalPlanner(tf2_ros::Buffer *tfBuffer_, string node_name_)
{
    //The tf buffer is used to lookup the base link position(tf from world frame to robot base frame)
    tfBuffer = tfBuffer_;
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));
    global_costmap_ptr.reset(new costmap_2d::Costmap2DROS("global_costmap", *tf_list_ptr)); //In ros kinetic the constructor uses tf instead of tf2 :(
    node_name = node_name_;
    configParams();
    configTopics();
    configServices();
    //Pase parameters from configParams to thetastar object
    configTheta();
}
bool GlobalPlanner::resetCostmapSrvCb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &rep)
{
    resetGlobalCostmap();
    return true;
}
void GlobalPlanner::resetGlobalCostmap()
{
    //Lock costmap so others threads cannot modify it
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_ptr->getCostmap()->getMutex()));
    global_costmap_ptr->resetLayers();
    lock.unlock();
}
void GlobalPlanner::configTheta()
{
    gbPlanner.initAuto(node_name, world_frame, goal_weight, cost_weight, lof_distance, &nh_);
    gbPlanner.setTimeOut(10);
    gbPlanner.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);
}
void GlobalPlanner::configTopics()
{
    trj_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory_tracker/input_trajectory", 1);
    replan_status_pub = nh_.advertise<std_msgs::Bool>("global_planner_node/replanning_status", 1);
    visMarkersPublisher = nh_.advertise<visualization_msgs::Marker>("global_planner_node/markers", 2);
}
void GlobalPlanner::configServices()
{
    reset_global_costmap_service = nh_.advertiseService("/global_planner_node/reset_costmap", &GlobalPlanner::resetCostmapSrvCb, this);

    rot_in_place_client_ptr.reset(new RotationInPlaceClient("Recovery_Rotation", true));
    execute_path_client_ptr.reset(new ExecutePathClient("Execute_Plan", true));

    //TODO: Removed the comments
    //execute_path_client_ptr->waitForServer();
    //rot_in_place_client_ptr->waitForServer();
    ROS_INFO_COND(debug, "Action client from global planner ready");

    //MakePlan MAIN Server Configuration
    make_plan_server_ptr.reset(new MakePlanServer(nh_, "Make_Plan", false));
    make_plan_server_ptr->registerGoalCallback(boost::bind(&GlobalPlanner::makePlanGoalCB, this));
    make_plan_server_ptr->registerPreemptCallback(boost::bind(&GlobalPlanner::makePlanPreemptCB, this));
    make_plan_server_ptr->start();
}
void GlobalPlanner::dynReconfCb(theta_star_2d::GlobalPlannerConfig &config, uint32_t level)
{
    this->cost_weight = config.cost_weight;
    this->lof_distance = config.lof_distance;
    this->goal_weight = config.goal_weight;
    this->occ_threshold = config.occ_threshold;

    gbPlanner.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: Dynamic reconfigure requested");
}
//This function gets parameter from param server at startup if they exists, if not it passes default values
void GlobalPlanner::configParams()
{
    //At startup, no goal and no costmap received yet
    goalRunning = false;
    nbrRotationsExec = 0;
    seq = 0;
    //Get params from param server. If they dont exist give variables default values
    nh_.param("/global_planner_node/show_config", showConfig, (bool)0);
    nh_.param("/global_planner_node/debug", debug, (bool)0);

    nh_.param("/global_planner_node/goal_weight", goal_weight, (float)1.5);
    nh_.param("/global_planner_node/cost_weight", cost_weight, (float)0.2);
    nh_.param("/global_planner_node/lof_distance", lof_distance, (float)0.2);
    nh_.param("/global_planner_node/occ_threshold", occ_threshold, (float)99);

    nh_.param("/global_planner_node/traj_dxy_max", traj_dxy_max, (float)1);
    nh_.param("/global_planner_node/traj_pos_tol", traj_pos_tol, (float)1);
    nh_.param("/global_planner_node/traj_yaw_tol", traj_yaw_tol, (float)0.1);

    nh_.param("/global_planner_node/world_frame", world_frame, (string) "/map");
    nh_.param("/global_planner_node/robot_base_frame", robot_base_frame, (string) "/base_link");

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner Node Configuration:");
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* with optim.: goal_weight = [%.2f]", goal_weight);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Occupied threshold = %.0f", occ_threshold);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Cost weight = [%.2f]", cost_weight);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Line of sight restriction = [%.2f]", lof_distance);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t World frame: %s, Robot base frame: %s", world_frame.c_str(), robot_base_frame.c_str());

    //Line strip marker use member points, only scale.x is used to control linea width
    lineMarker.header.frame_id = world_frame;
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.id = rand();
    lineMarker.ns = "global_path";
    lineMarker.lifetime = ros::Duration(500);
    lineMarker.type = RVizMarker::LINE_STRIP;
    lineMarker.action = RVizMarker::ADD;
    lineMarker.pose.orientation.w =1;
    lineMarker.color.r = 0.0;
    lineMarker.color.g = 1.0;
    lineMarker.color.b = 0.0;
    lineMarker.color.a = 1.0;
    lineMarker.scale.x = 0.1;

    waypointsMarker.header.frame_id = world_frame;
    waypointsMarker.header.stamp = ros::Time::now();
    waypointsMarker.ns = "global_path";
    waypointsMarker.id = lineMarker.id+1;
    waypointsMarker.lifetime = ros::Duration(500);
    waypointsMarker.type = RVizMarker::POINTS;
    waypointsMarker.action = RVizMarker::ADD;
    waypointsMarker.pose.orientation.w=1;
    waypointsMarker.color.r = 1.0;
    waypointsMarker.color.g = 1.0;
    waypointsMarker.color.b = 0.0;
    waypointsMarker.color.a = 1.0;
    waypointsMarker.scale.x = 0.15;
    waypointsMarker.scale.y = 0.15;
    waypointsMarker.scale.z = 0.4;

    //Configure map parameters
    map_resolution = global_costmap_ptr->getCostmap()->getResolution();
    ws_x_max = global_costmap_ptr->getCostmap()->getSizeInMetersX();
    ws_y_max = global_costmap_ptr->getCostmap()->getSizeInMetersY();
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: ws_x_max,ws_y_max, map_resolution: [%.2f, %.2f, %.2f]", ws_x_max, ws_y_max, map_resolution);
    gbPlanner.loadMapParams(ws_x_max, ws_y_max, map_resolution);
}
void GlobalPlanner::sendPathToLocalPlannerServer(trajectory_msgs::MultiDOFJointTrajectory path_)
{
    //Take the calculated path, insert it into an action, in the goal (ExecutePath.action)
    upo_actions::ExecutePathGoal goal_action;
    goal_action.path = path_;
    execute_path_client_ptr->sendGoal(goal_action);
}
void GlobalPlanner::publishMakePlanFeedback()
{
    float x, y;
    x = (getRobotPose().transform.translation.x - goal.vector.x );
    y = (getRobotPose().transform.translation.y - goal.vector.y );

    dist2Goal.data = sqrtf(x*x+y*y);
    make_plan_fb.distance_to_goal = dist2Goal;
    
    travel_time.data = (ros::Time::now() - start_time);
    make_plan_fb.travel_time = travel_time;
    /**
     *! We can start with the top speed and the first information is t = e/v with v=linear_max_speed
     *! from the param server and e the length of the path. 
     *? Then with the info RETURNED by the local planner or indirecly by the tracker about distance travelled
     *? (not in straight line) and the duration of the travel till the moment -> Calculate average speed 
     *TODO: When the tracker or local planner reports that they could not continue, either because the path
     *TODO: is blocked, the goal is occupied, there is a emergency stop, etc. -> Set estimation to infinty
    **/
    float speed;
    nh_.param("/nav_node/linear_max_speed", speed, (float)0.4);
    float time = pathLength / speed;
    int m= 0;
    float s;
    if (time < 60)
    {
        s = time;
    }
    else
    {
        while (time > 60)
        {
            time -= 60;
            m++;
        }
        s = time;
    }
    string data_ = "Estimated Time of Arrival " + to_string(m)+string(":")+to_string((int)s);
    make_plan_fb.ETA.data = data_;

    /**
     * ! To start you can calculate the CLOSEST global WAYPOINT, gave the total number of waypoints you 
     * ! can easily calculate the fraction of the path 
     * 
    **/
    float totalWaypoints = trajectory.points.size();
    float currentWaypoint = getClosestWaypoint();
    globalWaypoint.data = currentWaypoint;
    make_plan_fb.global_waypoint = globalWaypoint;
    float percent = (float)(currentWaypoint / totalWaypoints) * 100;
    data_ = to_string(percent) + string("%");
    make_plan_fb.percent_achieved.data = data_;

    make_plan_server_ptr->publishFeedback(make_plan_fb);
   
}
int GlobalPlanner::getClosestWaypoint()
{

    geometry_msgs::Transform robotPose = getRobotPose().transform;
    int waypoint = 0;
    float dist = std::numeric_limits<float>::max();
    float robotDist;
    for (auto it = trajectory.points.cbegin(); it != trajectory.points.cend(); it++)
    {
        robotDist = sqrtf(((robotPose.translation.x - it->transforms[0].translation.x) * (robotPose.translation.x - it->transforms[0].translation.x) +
                           (robotPose.translation.y - it->transforms[0].translation.y) * (robotPose.translation.y - it->transforms[0].translation.y)));

        if (robotDist < dist)
        {
            waypoint++;
            dist = robotDist;
        }
    }
    return waypoint;
}
void GlobalPlanner::makePlanGoalCB()
{
    //Cancel previous executing plan
    execute_path_client_ptr->cancelAllGoals();
    clearMarkers();
    goalRunning = true;
    nbrRotationsExec = 0;
    countImpossible = 0;
    make_plan_res.replan_number.data=0;

    start_time = ros::Time::now();
    upo_actions::MakePlanGoalConstPtr goal_ptr = make_plan_server_ptr->acceptNewGoal();

    goalPoseStamped = goal_ptr->global_goal;

    goal.vector.x = goalPoseStamped.pose.position.x;
    goal.vector.y = goalPoseStamped.pose.position.y;
    goal.header = goalPoseStamped.header;

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_ptr->getCostmap()->getMutex()));
    gbPlanner.getMap(global_costmap_ptr->getCostmap()->getCharMap());
    lock.unlock();

    ROS_INFO_COND(debug, "Called make plan srv");

    if (calculatePath())
    {
        ROS_INFO_COND(debug, "Succesfully calculated path");
        sendPathToLocalPlannerServer(trajectory);
    }
    else
    { // What if it isnt possible to calculate path?

        make_plan_res.not_possible = true;
        make_plan_res.finished = false;
        make_plan_server_ptr->setAborted(make_plan_res, "Impossible to calculate a solution");
        //execute_path_client_ptr->cancelAllGoals();
    }
}
void GlobalPlanner::makePlanPreemptCB()
{
    make_plan_res.finished = false;
    travel_time.data = ros::Time::now() - start_time;
    make_plan_res.time_spent = travel_time;

    make_plan_server_ptr->setPreempted(make_plan_res, "Goal Preempted by User");
}
bool GlobalPlanner::replan()
{
    resetGlobalCostmap();
    usleep(5e5);

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_ptr->getCostmap()->getMutex()));
    gbPlanner.getMap(global_costmap_ptr->getCostmap()->getCharMap());
    lock.unlock();
    make_plan_res.replan_number.data++;
    
    //For the world to know the planner is replanning
    flg_replan_status.data = true;
    replan_status_pub.publish(flg_replan_status);

    if (calculatePath())
    {
        ROS_INFO_COND(debug, "Succesfully calculated path");
        sendPathToLocalPlannerServer(trajectory);
        return true;
    }
    else
    {
        upo_actions::MakePlanResult result;
        make_plan_res.finished = false;
        make_plan_res.not_possible = true;
        make_plan_server_ptr->setAborted(make_plan_res, "Tried to replan and aborted after replanning and rotation in place");
        execute_path_client_ptr->cancelAllGoals();
        return false;
    }
}
void GlobalPlanner::clearMarkers(){

    waypointsMarker.action = RVizMarker::DELETEALL;
    lineMarker.action = RVizMarker::DELETEALL;
    
    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);
    
    lineMarker.points.clear();
    waypointsMarker.points.clear();
    
    lineMarker.action = RVizMarker::ADD;
    waypointsMarker.action = RVizMarker::ADD;
}
/**
 * This is the main function executed in loop
 * 
 **/
void GlobalPlanner::plan()
{
    //TODO Maybe I can change the goalRunning flag by check if is active any goal

    if (make_plan_server_ptr->isActive())
        publishMakePlanFeedback();

    if (goalRunning && execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::ABORTED) //!It means that local planner couldn t find a local solution
        replan();

    if (goalRunning && execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        clearMarkers();
        goalRunning = false;
        make_plan_res.finished = true;
        make_plan_res.not_possible = false;
        make_plan_res.time_spent.data = (ros::Time::now() - start_time);
        make_plan_server_ptr->setSucceeded(make_plan_res);
    }
}
bool GlobalPlanner::calculatePath()
{
    //It seems that you can get a goal and no map and try to get a path but setGoal and setStart will check if the points are valid
    //so if there is no map received it won't calculate a path
    bool ret = false;

    while (!ret)
    {
        if (setGoal() && setStart())
        {
            ROS_INFO_COND(debug, PRINTF_MAGENTA "Goal and start successfull set");
            // Path calculation

            ftime(&start);
            number_of_points = gbPlanner.computePath();
            ftime(&finish);

            seconds = finish.time - start.time - 1;
            milliseconds = (1000 - start.millitm) + finish.millitm;

            ROS_INFO(PRINTF_YELLOW "Time Spent in Global Path Calculation: %f ms", milliseconds + seconds * 1000);
            ROS_INFO(PRINTF_YELLOW "Number of points: %d", number_of_points);

            if (number_of_points > 0)
            {
                ROS_INFO_COND(debug, PRINTF_MAGENTA "Publishing trajectory, %d", number_of_points);
                publishTrajectory();

                //Reset the counter of the number of times the planner tried to calculate a path without success
                countImpossible = 0;
                //If it was replanning before, reset flag
                if (flg_replan_status.data)
                {
                    flg_replan_status.data = false;
                    replan_status_pub.publish(flg_replan_status);
                }

                ret = true;
            }
            else
            {
                if(nbrRotationsExec || flg_replan_status.data){
                    break;
                }
                
                countImpossible++;
                if (countImpossible == 3 && nbrRotationsExec < 1)
                {
                    rot_in_place_goal.execute_rotation = true;
                    rot_in_place_client_ptr->sendGoal(rot_in_place_goal);
                    rot_in_place_client_ptr->waitForResult(ros::Duration(15));
                    nbrRotationsExec++;
                }
            }
        }
        else
        {
            break;
        }
    }
    return ret;
}
void GlobalPlanner::calculatePathLength()
{
    float x, y;
    pathLength = 0;
    for (size_t it = 0; it < (trajectory.points.size() - 1); it++)
    {
        x = trajectory.points[it].transforms[0].translation.x - trajectory.points[it + 1].transforms[0].translation.x;
        y = trajectory.points[it].transforms[0].translation.y - trajectory.points[it + 1].transforms[0].translation.y;
        pathLength += sqrtf(x * x + y * y);
    }
}
geometry_msgs::TransformStamped GlobalPlanner::getRobotPose()
{
    geometry_msgs::TransformStamped ret;

    try
    {
        ret = tfBuffer->lookupTransform(world_frame, robot_base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Global Planner: Couldn't get Robot Pose, so not possible to set start point; tf exception: %s", ex.what());
    }
    return ret;
}
void GlobalPlanner::publishTrajectory()
{

    trajectory.joint_names.push_back(world_frame);
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = world_frame;
    trajectory.header.seq = ++seq;
    trajectory.points.clear();

    ROS_INFO_COND(debug, PRINTF_MAGENTA "Trajectory calculation...");

    geometry_msgs::TransformStamped transform_robot_pose = getRobotPose();

    if (number_of_points > 1)
    {
        gbPlanner.getTrajectoryYawInAdvance(trajectory, transform_robot_pose.transform);
    }
    else if (number_of_points == 1)
    {
        gbPlanner.getTrajectoryYawFixed(trajectory, gbPlanner.getYawFromQuat(transform_robot_pose.transform.rotation));
        trajectory_msgs::MultiDOFJointTrajectoryPoint pos;
        pos.transforms.push_back(transform_robot_pose.transform);
        trajectory.points.push_back(pos);
    }

    // Send the trajectory
    // Trajectory solution visualization marker

    trajectory_msgs::MultiDOFJointTrajectoryPoint goal_multidof;
    geometry_msgs::Transform transform_goal;

    transform_goal.rotation = goalPoseStamped.pose.orientation;
    transform_goal.translation.x = goalPoseStamped.pose.position.x;
    transform_goal.translation.y = goalPoseStamped.pose.position.y;
    goal_multidof.transforms.resize(1, transform_goal);

    trajectory.points.push_back(goal_multidof);

    trj_pub.publish(trajectory);

    //!Calculate path length:
    calculatePathLength();

    clearMarkers();

    lineMarker.header.stamp = ros::Time::now();
    waypointsMarker.header.stamp = ros::Time::now();

    geometry_msgs::Point p;
    
    for (size_t i = 0; i < trajectory.points.size(); i++)
    {
        p.x = trajectory.points[i].transforms[0].translation.x;
        p.y = trajectory.points[i].transforms[0].translation.y;
        
        lineMarker.points.push_back(p);
        waypointsMarker.points.push_back(p);
    }
   
    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);
}
bool GlobalPlanner::setGoal()
{
    bool ret = false;
    if (gbPlanner.setValidFinalPosition(goal.vector))
    {
        ret = true;
    }
    else
    {
        ROS_ERROR("Global Planner: Failed to set final global position: [%.2f, %.2f] ", goal.vector.x, goal.vector.y);
    }
    return ret;
}
bool GlobalPlanner::setStart()
{
    bool ret = false;
    geometry_msgs::Vector3Stamped start;
    start.vector.x = getRobotPose().transform.translation.x;
    start.vector.y = getRobotPose().transform.translation.y;

    if (gbPlanner.setValidInitialPosition(start.vector))
    {
        ret = true;
    }
    else if (gbPlanner.searchInitialPosition2d(0.25))
    {
        ROS_INFO(PRINTF_MAGENTA "Global Planner: Found a free initial position");
        ret = true;
    }
    else
    {
        ROS_ERROR("Global Planner: Failed to set initial global position(after search around): [%.2f, %.2f]", start.vector.x, start.vector.y);
    }
    return ret;
}
} // namespace PathPlanners