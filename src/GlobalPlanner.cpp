/*

*/

#include <theta_star/GlobalPlanner.hpp>

namespace PathPlanners
{
// Uncomment to set length catenary in nodes
#define USE_CATENARY_COMPUTE

GlobalPlanner::GlobalPlanner(string node_name_)
{
    //The tf buffer is used to lookup the base link position(tf from world frame to robot base frame)
    //tfBuffer = tfBuffer_;
    node_name = node_name_;

    nh.reset(new ros::NodeHandle("~"));
    nh->param("mode3d", use3d, (bool)false);

    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
    if (!use3d)
    {
#ifdef NOETIC
        global_costmap_ptr.reset(new costmap_2d::Costmap2DROS("global_costmap", *tfBuffer.get()));
#endif

#ifndef NOETIC
        tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));
        global_costmap_ptr.reset(new costmap_2d::Costmap2DROS("global_costmap", *tf_list_ptr)); //In ros kinetic the constructor uses tf instead of tf2 :(
#endif
        configParams2D();
    }
    else
    {
        tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));
        configParams3D();
    }

    configTopics();
    configServices();
    //Pase parameters from configParams to thetastar object
    configTheta();

    nh->param("use_catenary", use_catenary, (bool)false);
    nh->param("use_search_pyramid", use_search_pyramid, (bool)false);
    configCatenary();
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
    if (use3d)
    {
        theta3D.init(node_name, world_frame, ws_x_max, ws_y_max, ws_z_max, ws_x_min, ws_y_min, ws_z_min, map_resolution, map_h_inflaction, map_v_inflaction, goal_weight, z_weight_cost, z_not_inflate, nh);
        theta3D.setTimeOut(timeout);
        theta3D.setTrajectoryParams(traj_dxy_max, traj_dz_max, traj_pos_tol, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_wyaw_m, traj_yaw_tol);
        theta3D.confPrintRosWarn(false);
    }
    else
    {
        theta2D.initAuto(node_name, world_frame, goal_weight, cost_weight, lof_distance, nh);
        theta2D.setTimeOut(timeout);
        theta2D.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);
    }
}
void GlobalPlanner::configTopics()
{

    replan_status_pub = nh->advertise<std_msgs::Bool>("replanning_status", 1);
    visMarkersPublisher = nh->advertise<visualization_msgs::Marker>("markers", 2);
    fullRayPublisher = nh->advertise<visualization_msgs::Marker>("full_ray", 2);
    rayCastFreePublisher = nh->advertise<visualization_msgs::Marker>("ray_cast_free", 2);
    rayCastFreeReducedPublisher = nh->advertise<visualization_msgs::Marker>("ray_cast_free_reduced", 2);
    rayCastCollPublisher = nh->advertise<visualization_msgs::Marker>("ray_cast_coll", 2);
    rayCastNoFreePublisher = nh->advertise<visualization_msgs::Marker>("ray_cast_no_free", 2);
    reducedMapPublisher = nh->advertise<octomap_msgs::Octomap>("octomap_reduced", 1000);


    bool useOctomap;
    nh->param("use_octomap", useOctomap, (bool)false);

    if (use3d)
    {
        if (useOctomap)
        {
            sub_map = nh->subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &GlobalPlanner::collisionMapCallBack, this);
        }
        else
        {
            sub_map = nh->subscribe<PointCloud>("/points", 1, &GlobalPlanner::pointsSub, this);
        }
    }
}
void GlobalPlanner::collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    map = msg;
    theta3D.updateMap(map);
    mapRec = true;
    sub_map.shutdown();

    //ROS_INFO_COND(debug, PRINTF_MAGENTA "Collision Map Received");
}
void GlobalPlanner::pointsSub(const PointCloud::ConstPtr &points)
{
    if(mapRec){
        return;
    }
        
    std::string *error;
    if(!tf_list_ptr->canTransform(world_frame, points->header.frame_id,ros::Time::now(), error)){
        ROS_ERROR("Can't transform map: %s", error->c_str());
        return;
    }
    
    if (points->header.frame_id != world_frame)
    {
        PointCloud out;
        try
        {
            tf_list_ptr->waitForTransform(points->header.frame_id, world_frame, ros::Time::now(), ros::Duration(5));
            pcl_ros::transformPointCloud(world_frame, *points, out, *tf_list_ptr);
            ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner 3D: Collision Map Received after transform from %s to %s", points->header.frame_id.c_str(), world_frame.c_str());
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Transform exception: %s", ex.what());
            return;
        }
        theta3D.updateMap(out);
        mapRec = true;

    }
    else
    {
        theta3D.updateMap(*points);
        mapRec = true;
        ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner 3D: Collision Map Received");
    }
    theta3D.publishOccupationMarkersMap();


}
void GlobalPlanner::configServices()
{
    execute_path_client_ptr.reset(new ExecutePathClient("/Execute_Plan", true));
    make_plan_server_ptr.reset(new MakePlanServer(*nh, "/Make_Plan", false));
    make_plan_server_ptr->registerGoalCallback(boost::bind(&GlobalPlanner::makePlanGoalCB, this));
    make_plan_server_ptr->registerPreemptCallback(boost::bind(&GlobalPlanner::makePlanPreemptCB, this));

    make_plan_server_ptr->start();
    execute_path_client_ptr->waitForServer();

    if (!use3d)
    {
        reset_global_costmap_service = nh->advertiseService("reset_costmap", &GlobalPlanner::resetCostmapSrvCb, this);
        rot_in_place_client_ptr.reset(new RotationInPlaceClient("/Recovery_Rotation", true));

        rot_in_place_client_ptr->waitForServer();
    }

    ROS_INFO_COND(debug, "Global Planner 3D: Action client from global planner ready");
}
void GlobalPlanner::dynReconfCb(theta_star_2d::GlobalPlannerConfig &config, uint32_t level)
{
    if (!use3d)
    {
        this->cost_weight = config.cost_weight;
        this->lof_distance = config.lof_distance;
        this->goal_weight = config.goal_weight;
        this->occ_threshold = config.occ_threshold;

        theta2D.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);
        ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner 2D: Dynamic reconfigure requested");
    }
}
//This function gets parameter from param server at startup if they exists, if not it passes default values
void GlobalPlanner::configParams2D()
{
    //At startup, no goal and no costmap received yet
    nbrRotationsExec = 0;
    seq = 0;
    timesReplaned = 0;
    minPathLenght=0;
    //Get params from param server. If they dont exist give variables default values
    nh->param("show_config", showConfig, (bool)0);
    nh->param("debug", debug, (bool)0);
    nh->param("timeout", timeout, (double)10);
    nh->param("initial_position_search_dist", initialSearchAround, (double)0.25);

    nh->param("min_path_lenght", minPathLenght, (float)0.2);
    nh->param("goal_weight", goal_weight, (double)1.5);
    nh->param("cost_weight", cost_weight, (float)0.2);
    nh->param("lof_distance", lof_distance, (float)0.2);
    nh->param("occ_threshold", occ_threshold, (float)99);

    nh->param("traj_dxy_max", traj_dxy_max, (double)1);
    nh->param("traj_pos_tol", traj_pos_tol, (double)1);
    nh->param("traj_yaw_tol", traj_yaw_tol, (double)0.1);

    nh->param("world_frame", world_frame, (string) "/map");
    nh->param("robot_base_frame", robot_base_frame, (string) "/base_link");

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner Node Configuration:");
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* with optim.: goal_weight = [%.2f]", goal_weight);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Occupied threshold = %.0f", occ_threshold);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Cost weight = [%.2f]", cost_weight);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Line of sight restriction = [%.2f]", lof_distance);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t World frame: %s, Robot base frame: %s", world_frame.c_str(), robot_base_frame.c_str());

    //Line strip marker use member points, only scale.x is used to control linea width

    configMarkers("global_path_2d");

    //Configure map parameters
    map_resolution = global_costmap_ptr->getCostmap()->getResolution();
    ws_x_max = global_costmap_ptr->getCostmap()->getSizeInMetersX();
    ws_y_max = global_costmap_ptr->getCostmap()->getSizeInMetersY();
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner 2D: ws_x_max,ws_y_max, map_resolution: [%.2f, %.2f, %.2f]", ws_x_max, ws_y_max, map_resolution);
    theta2D.loadMapParams(ws_x_max, ws_y_max, map_resolution);
}

void GlobalPlanner::configMarkers(std::string ns)
{

    lineMarker.ns = ns;
    lineMarker.header.frame_id = world_frame;
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.id = rand();
    lineMarker.lifetime = ros::Duration(500);
    lineMarker.type = RVizMarker::LINE_STRIP;
    lineMarker.action = RVizMarker::ADD;
    lineMarker.pose.orientation.w = 1;
    lineMarker.color.r = 0.0;
    lineMarker.color.g = 1.0;
    lineMarker.color.b = 0.0;
    lineMarker.color.a = 1.0;
    lineMarker.scale.x = 0.1;

    waypointsMarker.ns = ns;
    waypointsMarker.header.frame_id = world_frame;
    waypointsMarker.header.stamp = ros::Time::now();
    waypointsMarker.id = lineMarker.id + 1;
    waypointsMarker.lifetime = ros::Duration(500);
    waypointsMarker.type = RVizMarker::POINTS;
    waypointsMarker.action = RVizMarker::ADD;
    waypointsMarker.pose.orientation.w = 1;
    waypointsMarker.color.r = 1.0;
    waypointsMarker.color.g = 1.0;
    waypointsMarker.color.b = 0.0;
    waypointsMarker.color.a = 1.0;
    waypointsMarker.scale.x = 0.15;
    waypointsMarker.scale.y = 0.15;
    // waypointsMarker.scale.z = 0.4;

    fullrayMarker.ns = ns;
    fullrayMarker.header.frame_id = world_frame;
    fullrayMarker.header.stamp = ros::Time::now();
    fullrayMarker.id = lineMarker.id + 1;
    fullrayMarker.lifetime = ros::Duration(500);
    fullrayMarker.type = RVizMarker::POINTS;
    fullrayMarker.action = RVizMarker::ADD;
    fullrayMarker.pose.orientation.w = 1;
    fullrayMarker.color.r = 1.0;
    fullrayMarker.color.g = 1.0;
    fullrayMarker.color.b = 1.0;
    fullrayMarker.color.a = 1.0;
    fullrayMarker.scale.x = 0.05;
    fullrayMarker.scale.y = 0.05;
    // fullrayMarker.scale.z = 0.05;
    
    raycastfreeMarker.ns = ns;
    raycastfreeMarker.header.frame_id = world_frame;
    // raycastfreeMarker.header.stamp = ros::Time::now();
    raycastfreeMarker.id = lineMarker.id + 1;
    raycastfreeMarker.lifetime = ros::Duration(500);
    raycastfreeMarker.type = RVizMarker::POINTS;
    raycastfreeMarker.action = RVizMarker::ADD;
    raycastfreeMarker.pose.orientation.w = 1;
    raycastfreeMarker.color.r = 0.6;
    raycastfreeMarker.color.g = 0.6;
    raycastfreeMarker.color.b = 0.6;
    raycastfreeMarker.color.a = 1.0;
    raycastfreeMarker.scale.x = 0.05;
    raycastfreeMarker.scale.y = 0.05;
    // raycastfreeMarker.scale.z = 0.05;

    raycastfreereducedMarker.ns = ns;
    raycastfreereducedMarker.header.frame_id = world_frame;
    // raycastfreereducedMarker.header.stamp = ros::Time::now();
    raycastfreereducedMarker.id = lineMarker.id + 1;
    raycastfreereducedMarker.lifetime = ros::Duration(500);
    raycastfreereducedMarker.type = RVizMarker::POINTS;
    raycastfreereducedMarker.action = RVizMarker::ADD;
    raycastfreereducedMarker.pose.orientation.w = 1;
    raycastfreereducedMarker.color.r = 1.0;
    raycastfreereducedMarker.color.g = 1.0;
    raycastfreereducedMarker.color.b = 0.0;
    raycastfreereducedMarker.color.a = 1.0;
    raycastfreereducedMarker.scale.x = 0.05;
    raycastfreereducedMarker.scale.y = 0.05;
    // raycastfreereducedMarker.scale.z = 0.05;

    raycastcollMarker.ns = ns;
    raycastcollMarker.header.frame_id = world_frame;
    // raycastcollMarker.header.stamp = ros::Time::now();
    raycastcollMarker.id = lineMarker.id + 1;
    raycastcollMarker.lifetime = ros::Duration(500);
    raycastcollMarker.type = RVizMarker::POINTS;
    raycastcollMarker.action = RVizMarker::ADD;
    raycastcollMarker.pose.orientation.w = 1;
    raycastcollMarker.color.r = 0.6;
    raycastcollMarker.color.g = 0.6;
    raycastcollMarker.color.b = 0.0;
    raycastcollMarker.color.a = 1.0;
    raycastcollMarker.scale.x = 0.05;
    raycastcollMarker.scale.y = 0.05;
    // raycastcollMarker.scale.z = 0.05;

    raycastnofreeMarker.ns = ns;
    raycastnofreeMarker.header.frame_id = world_frame;
    // raycastnofreeMarker.header.stamp = ros::Time::now();
    raycastnofreeMarker.id = lineMarker.id + 1;
    raycastnofreeMarker.lifetime = ros::Duration(500);
    raycastnofreeMarker.type = RVizMarker::POINTS;
    raycastnofreeMarker.action = RVizMarker::ADD;
    raycastnofreeMarker.pose.orientation.w = 1;
    raycastnofreeMarker.color.r = 0.3;
    raycastnofreeMarker.color.g = 0.3;
    raycastnofreeMarker.color.b = 0.2;
    raycastnofreeMarker.color.a = 1.0;
    raycastnofreeMarker.scale.x = 0.05;
    raycastnofreeMarker.scale.y = 0.05;
    // raycastfreenoMarker.scale.z = 0.05;
}

void GlobalPlanner::configParams3D()
{
    //At startup, no goal and no costmap received yet
    nbrRotationsExec = 0;
    seq = 0;
    timesReplaned = 0;
    mapRec = false;
    minPathLenght=0;

    //Get params from param server. If they dont exist give variables default values
    nh->param("show_config", showConfig, (bool)0);
    nh->param("debug", debug, (bool)0);
    nh->param("timeout", timeout, (double)10);
    nh->param("initial_position_search_dist", initialSearchAround, (double)1);

    nh->param("ws_x_max", ws_x_max, (double)30);
    nh->param("ws_y_max", ws_y_max, (double)30);
    nh->param("ws_z_max", ws_z_max, (double)30);
    nh->param("ws_x_min", ws_x_min, (double)0);
    nh->param("ws_y_min", ws_y_min, (double)0);
    nh->param("ws_z_min", ws_z_min, (double)0);

    nh->param("map_resolution", map_resolution, (double)0.05);
    nh->param("map_h_inflaction", map_h_inflaction, (double)0.05);
    nh->param("map_v_inflaction", map_v_inflaction, (double)0.05);

    nh->param("z_weight_cost", z_weight_cost, (double)1.2);
    nh->param("z_not_inflate", z_not_inflate, (double)8);
    nh->param("goal_weight", goal_weight, (double)1.1);

    nh->param("traj_dxy_max", traj_dxy_max, (double)1);
    nh->param("traj_pos_tol", traj_pos_tol, (double)1);
    nh->param("traj_yaw_tol", traj_yaw_tol, (double)0.1);
    nh->param("traj_dz_max", traj_dz_max, (double)1);
    nh->param("traj_vxy_m", traj_vxy_m, (double)1);
    nh->param("traj_vz_m", traj_vz_m, (double)1);
    nh->param("traj_vxy_m_1", traj_vxy_m_1, (double)1);
    nh->param("traj_vz_m_1", traj_vz_m_1, (double)1);
    nh->param("traj_wyaw_m", traj_wyaw_m, (double)1);

    nh->param("world_frame", world_frame, (string) "/map");
    nh->param("robot_base_frame", robot_base_frame, (string) "/base_link");

  	nh->param("write_data_for_analysis",write_data_for_analysis, (bool)0);
	nh->param("path", path, (std::string) "~/");


    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D Node Configuration:");
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Workspace = X: [%.2f, %.2f]\t Y: [%.2f, %.2f]\t Z: [%.2f, %.2f]  ", ws_x_max, ws_x_min, ws_y_max, ws_y_min, ws_z_max, ws_z_min);

    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* with optim.: goal_weight = [%.2f]", goal_weight);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t World frame: %s, Robot base frame: %s", world_frame.c_str(), robot_base_frame.c_str());

    configMarkers("global_path_3d");
}

void GlobalPlanner::sendPathToLocalPlannerServer()
{
    //Take the calculated path, insert it into an action, in the goal (ExecutePath.action)
    upo_actions::ExecutePathGoal goal_action;
    printfTrajectory(trajectory, "sendPathToLocalPlannerServer: Trajectory_state_1");
    goal_action.path = trajectory;
    if (use_catenary){
        for(int i= 0; i<theta3D.length_catenary.size() ; i++){
            goal_action.length_catenary.push_back(theta3D.length_catenary[i]);
        }
    }

    execute_path_client_ptr->sendGoal(goal_action);
}

void GlobalPlanner::publishMakePlanFeedback()
{
    float x = 0, y = 0, z = 0;
    x = (getRobotPose().transform.translation.x - goal.vector.x);
    y = (getRobotPose().transform.translation.y - goal.vector.y);
    z = (getRobotPose().transform.translation.z - goal.vector.z);

    dist2Goal.data = sqrtf(x * x + y * y + z * z);
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
    nh->param("/nav_node/linear_max_speed", speed, (float)0.4);
    float time = pathLength / speed;
    int m = 0;
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
    string data_ = "Estimated Time of Arrival " + to_string(m) + string(":") + to_string((int)s);
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
                           (robotPose.translation.y - it->transforms[0].translation.y) * (robotPose.translation.y - it->transforms[0].translation.y) +
                           (robotPose.translation.z - it->transforms[0].translation.z) * (robotPose.translation.z - it->transforms[0].translation.z)));

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
    nbrRotationsExec = 0;
    countImpossible = 0;
    timesReplaned = 0;
    make_plan_res.replan_number.data = 0;

    start_time = ros::Time::now();
    upo_actions::MakePlanGoalConstPtr goal_ptr = make_plan_server_ptr->acceptNewGoal();

    goalPoseStamped = goal_ptr->global_goal;

    goal.vector.x = goalPoseStamped.pose.position.x;
    goal.vector.y = goalPoseStamped.pose.position.y;
    goal.vector.z = goalPoseStamped.pose.position.z;
    goal.header = goalPoseStamped.header;

    if (!use3d)
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_ptr->getCostmap()->getMutex()));
        theta2D.getMap(global_costmap_ptr->getCostmap()->getCharMap());
        lock.unlock();
    }

    ROS_INFO_COND(debug, "Global Planner: Called Make Plan");

    if (calculatePath())
    {
        ROS_INFO_COND(debug, "Global Planner: Succesfully calculated path");
        sendPathToLocalPlannerServer();
    }
    else
    { // What if it isnt possible to calculate path?
        make_plan_res.not_possible = true;
        make_plan_res.finished = false;
        make_plan_server_ptr->setAborted(make_plan_res, "Impossible to calculate a solution");
    }
}

void GlobalPlanner::makePlanPreemptCB()
{
    make_plan_res.finished = false;
    travel_time.data = ros::Time::now() - start_time;
    make_plan_res.time_spent = travel_time;

    make_plan_server_ptr->setPreempted(make_plan_res, "Goal Preempted by User");
    ROS_INFO("Global Planner: Make plan preempt cb: cancelling");
    execute_path_client_ptr->cancelAllGoals();
    clearMarkers();
}

bool GlobalPlanner::replan()
{
    if (!use3d)
    {
        resetGlobalCostmap();
        usleep(5e5);
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_ptr->getCostmap()->getMutex()));
        theta2D.getMap(global_costmap_ptr->getCostmap()->getCharMap());
        lock.unlock();
    }

    make_plan_res.replan_number.data++;

    //For the world to know the planner is replanning
    flg_replan_status.data = true;
    replan_status_pub.publish(flg_replan_status);

    bool ret_val = false;
    if (calculatePath())
    {
        ++timesReplaned;
        ROS_INFO_COND(debug, "Global Planner: Succesfully calculated path");
        sendPathToLocalPlannerServer();
        ret_val = true;
    }
    else if (timesReplaned > 5)
    {
        timesReplaned = 0;
        make_plan_res.finished = false;
        make_plan_res.not_possible = true;

        flg_replan_status.data = false;
        replan_status_pub.publish(flg_replan_status);

        make_plan_server_ptr->setAborted(make_plan_res, "Tried to replan and aborted after replanning 5 times");
        execute_path_client_ptr->cancelAllGoals();
    }
    return false;
}

void GlobalPlanner::clearMarkers()
{

    waypointsMarker.action = RVizMarker::DELETEALL;
    lineMarker.action = RVizMarker::DELETEALL;

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);

    lineMarker.points.clear();
    waypointsMarker.points.clear();

    lineMarker.action = RVizMarker::ADD;
    waypointsMarker.action = RVizMarker::ADD;
}

void GlobalPlanner::clearMarkersRayCast()
{
    fullrayMarker.action = RVizMarker::DELETEALL; 
    raycastfreeMarker.action = RVizMarker::DELETEALL;
    raycastfreereducedMarker.action = RVizMarker::DELETEALL; 
    raycastcollMarker.action = RVizMarker::DELETEALL; 
    raycastnofreeMarker.action = RVizMarker::DELETEALL;

    fullRayPublisher.publish(fullrayMarker); 
    rayCastFreePublisher.publish(raycastfreeMarker); 
    rayCastFreeReducedPublisher.publish(raycastfreereducedMarker); 
    rayCastCollPublisher.publish(raycastcollMarker); 
    rayCastNoFreePublisher.publish(raycastnofreeMarker); 
    
    fullrayMarker.points.clear();
    raycastfreeMarker.points.clear();
    raycastfreereducedMarker.points.clear();
    raycastcollMarker.points.clear();
    raycastnofreeMarker.points.clear();


    fullrayMarker.action = RVizMarker::ADD; 
    raycastfreeMarker.action = RVizMarker::ADD;
    raycastfreereducedMarker.action = RVizMarker::ADD; 
    raycastcollMarker.action = RVizMarker::ADD; 
    raycastnofreeMarker.action = RVizMarker::ADD;
}
/**
 * This is the main function executed in loop
 **/
void GlobalPlanner::plan()
{
    //TODO Maybe I can change the goalRunning flag by check if is active any goal

    if (make_plan_server_ptr->isActive())
    {
        publishMakePlanFeedback();
    }
    else
    {
        return;
    }

    if (execute_path_client_ptr->getState().isDone())
    {
        clearMarkers();

        if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            make_plan_res.finished = true;
            make_plan_res.not_possible = false;
            make_plan_res.replan_number.data = timesReplaned;
            make_plan_res.time_spent.data = (ros::Time::now() - start_time);
            make_plan_server_ptr->setSucceeded(make_plan_res);
        }

        if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_INFO("Global Planner: Path execution aborted by local planner...");
            replan();
        } //!It means that local planner couldn t find a local solution

        if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
        { //!Maybe when the goal is inside the local workspace and occupied

            ROS_INFO("Global Planner: Path execution preempted by local planner");
            replan();
            //Decide What to do next....
        }
        if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::REJECTED)
        { //!Maybe when the goal is inside the local workspace and occupied

            ROS_INFO("Global Planner: Path execution rejected by local planner");
            replan();
            //Decide What to do next....
        }
    }
}

bool GlobalPlanner::calculatePath()
{
    //It seems that you can get a goal and no map and try to get a path but setGoal and setStart will check if the points are valid
    //so if there is no map received it won't calculate a path
    bool ret = false;

    if (use3d && !mapRec)
        return ret;

    if (setGoal() && setStart(start_point))
    {
        //Reduced Map 
        clearMarkersRayCast();
        std::vector<octomap::point3d> v_full_ray, v_ray_cast_free, v_ray_cast_free_reduced, v_ray_cast_coll, v_no_ray_cast_free;
        octomap::OcTree map_reduced = theta3D.updateMapReduced(map, goal, start_point, v_full_ray, v_ray_cast_free, v_ray_cast_free_reduced, v_ray_cast_coll, v_no_ray_cast_free);
        octomap_msgs::Octomap octomap_reduced;
        octomap_reduced.binary = 1 ;
        octomap_reduced.id = 1 ;
        octomap_reduced.resolution =0.2 ;
        octomap_reduced.header.frame_id = "/map";
        octomap_reduced.header.stamp = ros::Time::now();
        octomap_msgs::fullMapToMsg(map_reduced, octomap_reduced);
        reducedMapPublisher.publish(octomap_reduced);
        geometry_msgs::Point p;
        fullrayMarker.header.stamp = ros::Time::now();
        for (size_t i = 0; i < v_full_ray.size(); i++)
        {
            p.x = v_full_ray[i].x();
            p.y = v_full_ray[i].y();
            p.z = v_full_ray[i].z();
            fullrayMarker.points.push_back(p);
        }
        raycastfreeMarker.header.stamp = ros::Time::now();
        for (size_t i = 0; i < v_ray_cast_free.size(); i++)
        {
            p.x = v_ray_cast_free[i].x();
            p.y = v_ray_cast_free[i].y();
            p.z = v_ray_cast_free[i].z();
            raycastfreeMarker.points.push_back(p);
        }
        raycastfreereducedMarker.header.stamp = ros::Time::now();
        for (size_t i = 0; i < v_ray_cast_free_reduced.size(); i++)
        {
            p.x = v_ray_cast_free_reduced[i].x();
            p.y = v_ray_cast_free_reduced[i].y();
            p.z = v_ray_cast_free_reduced[i].z();
            raycastfreereducedMarker.points.push_back(p);
        }
        raycastcollMarker.header.stamp = ros::Time::now();
        for (size_t i = 0; i < v_ray_cast_coll.size(); i++)
        {
            p.x = v_ray_cast_coll[i].x();
            p.y = v_ray_cast_coll[i].y();
            p.z = v_ray_cast_coll[i].z();
            raycastcollMarker.points.push_back(p);
        }
        raycastnofreeMarker.header.stamp = ros::Time::now();
        printf("============================ v_no_ray_cast_free.size()=[%lu]",v_no_ray_cast_free.size());
        for (size_t i = 0; i < v_no_ray_cast_free.size(); i++)
        {
            p.x = v_no_ray_cast_free[i].x();
            p.y = v_no_ray_cast_free[i].y();
            p.z = v_no_ray_cast_free[i].z();
            raycastnofreeMarker.points.push_back(p);
        }
        fullRayPublisher.publish(fullrayMarker);
        rayCastFreePublisher.publish(raycastfreeMarker);
        rayCastFreeReducedPublisher.publish(raycastfreereducedMarker);
        rayCastCollPublisher.publish(raycastcollMarker);
        rayCastNoFreePublisher.publish(raycastnofreeMarker);

        //Print succes start and goal points
        ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: Goal and start successfull set");
        
        // Path calculation
        // ftime(&start);
        clock_gettime(CLOCK_REALTIME, &start);
        if (use3d)
        {
            number_of_points = theta3D.computePath();
        }
        else
        {
            number_of_points = theta2D.computePath();
        }
        // ftime(&finish);
        clock_gettime(CLOCK_REALTIME, &finish);

        // seconds = finish.time - start.time - 1;
        // milliseconds = (1000 - start.millitm) + finish.millitm;
        seconds = finish.tv_sec - start.tv_sec - 1;
        milliseconds = (1000000000.0 - start.tv_nsec) + finish.tv_nsec;

        if (write_data_for_analysis){
            std::ofstream ofs;
            std::string output_file = path + "time_compute_initial_planner.txt";
            ofs.open(output_file.c_str(), std::ofstream::app);
            if (ofs.is_open()) {
                std::cout << "Saving time initial planning data in output file: " << output_file << std::endl;
                ofs << (milliseconds + seconds * 1000000000.0)/1000000000.0 <<std::endl;
            } 
            else {
                std::cout << "Couldn't be open the output data file for time initial planning" << std::endl;
            }
            ofs.close();
        }

        ROS_INFO(PRINTF_YELLOW "Global Planner: Time Spent in Global Path Calculation: %.1f ms", milliseconds + seconds * 1000000000.0);
        ROS_INFO(PRINTF_YELLOW "Global Planner: Number of points: %d", number_of_points);

        if (number_of_points > 0)
        {
            ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: Publishing trajectory");
            if (use3d)
            {
                publishTrajectory3D();
            }
            else
            {
                publishTrajectory2D();
            }

            if (pathLength < minPathLenght)
            {
                execute_path_client_ptr->cancelAllGoals();
                make_plan_server_ptr->setSucceeded();
            }
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
            countImpossible++;
        }
    }

    return ret;
}

void GlobalPlanner::calculatePathLength()
{
    float x, y, z;
    pathLength = 0;
    for (size_t it = 0; it < (trajectory.points.size() - 1); it++)
    {
        x = trajectory.points[it].transforms[0].translation.x - trajectory.points[it + 1].transforms[0].translation.x;
        y = trajectory.points[it].transforms[0].translation.y - trajectory.points[it + 1].transforms[0].translation.y;
        z = trajectory.points[it].transforms[0].translation.z - trajectory.points[it + 1].transforms[0].translation.z;

        pathLength += sqrtf(x * x + y * y + z * z);
    }
    ROS_INFO_COND(debug,PRINTF_MAGENTA"Global path lenght: %f, number of points: %d", pathLength, (int)trajectory.points.size());

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

void GlobalPlanner::publishTrajectory2D()
{

    trajectory.joint_names.push_back(world_frame);
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = world_frame;
    trajectory.header.seq = ++seq;
    trajectory.points.clear();

    ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner 2D: Trajectory calculation...");

    geometry_msgs::TransformStamped transform_robot_pose = getRobotPose();

    if (number_of_points > 1)
    {
        theta2D.getTrajectoryYawInAdvance(trajectory, transform_robot_pose.transform);
    }
    else if (number_of_points == 1)
    {
        theta2D.getTrajectoryYawFixed(trajectory, theta2D.getYawFromQuat(transform_robot_pose.transform.rotation));
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

void GlobalPlanner::publishTrajectory3D()
{
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = world_frame;
    trajectory.header.seq = ++seq;
    trajectory.points.clear();
    

    ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner 3D: Trajectory calculation... POINTS: %d",number_of_points);

    geometry_msgs::TransformStamped transform_robot_pose = getRobotPose();

    if (number_of_points > 1)
    {
        theta3D.getTrajectoryYawInAdvance(trajectory, transform_robot_pose.transform);
        // theta3D.getTrajectoryYawFixed(trajectory, theta3D.getYawFromQuat(transform_robot_pose.transform.rotation));

        if(use_catenary){
			double lengthToset;
            theta3D.length_catenary_aux.clear();
			for(size_t j=0 ; j < theta3D.length_catenary.size(); j++){
				theta3D.length_catenary_aux.push_back(theta3D.length_catenary[j]) ;
			}
			theta3D.length_catenary.clear();

			ThetaStarNode3D init_wp;
			init_wp.point.x= transform_robot_pose.transform.translation.x*theta3D.step_inv;
			init_wp.point.y= transform_robot_pose.transform.translation.y*theta3D.step_inv;
			init_wp.point.z= transform_robot_pose.transform.translation.z*theta3D.step_inv;
   
			if(theta3D.feasibleCatenary(init_wp, theta3D.tf_reel,transform_robot_pose.transform.translation))
				lengthToset = init_wp.lengthCatenary;
            
			for (size_t j = 0 ; j < theta3D.length_catenary_aux.size(); j++){
				if (j==0)
					theta3D.length_catenary.push_back(lengthToset);
				theta3D.length_catenary.push_back(theta3D.length_catenary_aux[j]);
			}

            for(size_t k= 0; k < theta3D.length_catenary.size(); k++)
                printf("theta3d.length_catenary.size()=[%lu] length=[%f]\n",theta3D.length_catenary.size(),theta3D.length_catenary[k]);			
        }
        

        printfTrajectory(trajectory, "After getting traj: ");
    }
    else if (number_of_points == 1)
    {
        theta3D.getTrajectoryYawFixed(trajectory, theta3D.getYawFromQuat(transform_robot_pose.transform.rotation));
        trajectory_msgs::MultiDOFJointTrajectoryPoint pos;
        // pos.transforms.push_back(transform_robot_pose.transform);
        // trajectory.points.push_back(pos);
    }

    // Send the trajectory
    // Trajectory solution visualization marker

    trajectory_msgs::MultiDOFJointTrajectoryPoint goal_multidof;
    geometry_msgs::Transform transform_goal;

    transform_goal.rotation = goalPoseStamped.pose.orientation;
    transform_goal.translation.x = goalPoseStamped.pose.position.x;
    transform_goal.translation.y = goalPoseStamped.pose.position.y;
    transform_goal.translation.z = goalPoseStamped.pose.position.z;

    goal_multidof.transforms.resize(1, transform_goal);

    // trajectory.points.push_back(goal_multidof);

    //!Calculate path length:
    calculatePathLength();

    clearMarkers();

    lineMarker.header.stamp = ros::Time::now();
    waypointsMarker.header.stamp = ros::Time::now();

    geometry_msgs::Point p;

    ROS_INFO("trajectory.points.size()=[%lu]",trajectory.points.size());
    for (size_t i = 0; i < trajectory.points.size(); i++)
    {
        p.x = trajectory.points[i].transforms[0].translation.x;
        p.y = trajectory.points[i].transforms[0].translation.y;
        p.z = trajectory.points[i].transforms[0].translation.z;

        lineMarker.points.push_back(p);
        waypointsMarker.points.push_back(p);
    }

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);
}

bool GlobalPlanner::setGoal()
{
    bool ret = false;
    if (use3d)
    {
        if (theta3D.setValidFinalPosition(goal.vector))
        {
            ret = true;
        }
        else
        {
            ROS_ERROR("Global Planner: Failed to set final global position: [%.2f, %.2f, %.2f] ", goal.vector.x, goal.vector.y, goal.vector.z);
        }
    }
    else
    {
        if (theta2D.setValidFinalPosition(goal.vector))
        {
            ret = true;
        }
        else
        {
            ROS_ERROR("Global Planner: Failed to set final global position: [%.2f, %.2f] ", goal.vector.x, goal.vector.y);
        }
    }

    return ret;
}

bool GlobalPlanner::setStart(geometry_msgs::Vector3Stamped &start)
{
    bool ret = false;
    start.vector.x = getRobotPose().transform.translation.x;
    start.vector.y = getRobotPose().transform.translation.y;
    start.vector.z = getRobotPose().transform.translation.z;

    if (start.vector.z <= ws_z_min)
        start.vector.z = ws_z_min + map_v_inflaction + map_resolution;

    if (use3d)
    {
        if (theta3D.setValidInitialPosition(start.vector))
        {
            ret = true;
        }
        else if (theta3D.searchInitialPosition3d(initialSearchAround))
        {
            ROS_INFO(PRINTF_MAGENTA "Global Planner 3D: Found a free initial position");
            ret = true;
        }
        else
        {
            ROS_ERROR("Global Planner 3D: Failed to set initial global position(after search around): [%.2f, %.2f, %.2f]", start.vector.x, start.vector.y, start.vector.z);
        }
    }
    else
    {
        if (theta2D.setValidInitialPosition(start.vector))
        {
            ret = true;
        }
        else if (theta2D.searchInitialPosition2d(initialSearchAround))
        {
            ROS_INFO(PRINTF_MAGENTA "Global Planner 2D: Found a free initial position");
            ret = true;
        }
        else
        {
            ROS_ERROR("Global Planner 2D: Failed to set initial global position(after search around): [%.2f, %.2f]", start.vector.x, start.vector.y);
        }
    }
    return ret;
}

void GlobalPlanner::configCatenary(){
    nh->param("multiplicative_factor", multiplicative_factor, (double)1.001);
    nh->param("bound_bisection_a", bound_bisection_a, (double)100.0);
    nh->param("bound_bisection_b", bound_bisection_b, (double)100.0);
    nh->param("length_tether_max", length_tether_max, (double)10.0);

	printf("GlobalPlanner::configCatenary :   USE_OF_CATENARY=[%s] VALUES=[%f %f %f %f] !!\n",use_catenary ? "true" : "false", multiplicative_factor, bound_bisection_a,bound_bisection_b, length_tether_max);
	printf("GlobalPlanner::configCatenary :   USE_SEARCH_PYRAMID=[%s] \n", use_search_pyramid ? "true" : "false");
    theta3D.configCatenaryCompute(use_catenary, use_search_pyramid, multiplicative_factor,bound_bisection_a,bound_bisection_b, length_tether_max,tfListenerReel());
}

geometry_msgs::Vector3 GlobalPlanner::tfListenerReel(){

    geometry_msgs::Vector3 _p_reel;
	tf::StampedTransform _transform;

    listener.lookupTransform("map", "reel_base_link", ros::Time(0), _transform);

	_p_reel.x = _transform.getOrigin().x();
	_p_reel.y = _transform.getOrigin().y();
	_p_reel.z = _transform.getOrigin().z();

	return _p_reel;
}

} // namespace PathPlanners
