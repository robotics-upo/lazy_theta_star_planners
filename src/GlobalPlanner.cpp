/*

*/

#include <theta_star/GlobalPlanner.hpp>

namespace PathPlanners
{
//TODO: use ros::NodeHandlePtr instead of classic pointer
GlobalPlanner::GlobalPlanner(tf2_ros::Buffer *tfBuffer_)
{
    //The tf buffer is used to lookup the base link position(tf from world frame to robot base frame)
    tfBuffer = tfBuffer_;
    configParams();
    configTopics();
    configServices();
    //Pase parameters from configParams to thetastar object
    configTheta();
}
void GlobalPlanner::configTheta()
{
    string node_name = "global_planner_node";
    gbPlanner.initAuto(node_name, world_frame, goal_weight, cost_weight, lof_distance, &nh_);
    gbPlanner.setTimeOut(20);
    gbPlanner.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);
}
void GlobalPlanner::configTopics()
{
    //The topic where the trajectory message will be published
    string topicPath;
    nh_.param("/global_planner_node/traj_topic", topicPath, (string) "trajectory_tracker/input_trajectory");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Global Planner: Trajectory output topic: %s", topicPath.c_str());
    trj_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 0);
    //Visualization topic for RViz marker
    nh_.param("/global_planner_node/vis_marker_traj_topic", topicPath, (string) "global_planner_node/visualization_marker_trajectory");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Global Planner: Visualization marker trajectory output topic: %s", topicPath.c_str());
    vis_trj_pub = nh_.advertise<visualization_msgs::MarkerArray>(topicPath, 0);

    //goal input topic
    nh_.param("/global_planner_node/goal_topic", topicPath, (string) "/move_base_simple/goal");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Global Planner: Goal input topic: %s", topicPath.c_str());
    goal_sub = nh_.subscribe<geometry_msgs::PoseStamped>(topicPath, 1, &GlobalPlanner::goalCb, this);

    //map input topic, it could a map from map server or a costmap
    nh_.param("/global_planner_node/global_map_topic", topicPath, (string) "/costmap_2d/costmap/costmap");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Global Planner: Global costmap input topic: %s", topicPath.c_str());
    global_costmap_sub = nh_.subscribe<nav_msgs::OccupancyGrid>(topicPath, 1, &GlobalPlanner::globalCostMapCb, this);
}
void GlobalPlanner::configServices()
{

    global_replanning_service = nh_.advertiseService("/global_planner_node/global_replanning_service", &GlobalPlanner::replanningSrvCb, this);
}
//This function gets parameter from param server at startup if they exists, if not it passes default values
void GlobalPlanner::configParams()
{
    //At startup, no goal and no costmap received yet
    globalGoalReceived = false;
    gCmReceived = false;
    mapParamsConfigured = false;
    //Get params from param server. If they dont exist give variables default values
    nh_.param("/global_planner_node/show_config", showConfig, (bool)0);
    nh_.param("/global_planner_node/debug", debug, (bool)0);

    //nh_.param("/global_planner_node/ws_x_max", ws_x_max, (float)6);
    //nh_.param("/global_planner_node/ws_y_max", ws_y_max, (float)6);
    //nh_.param("/global_planner_node/map_resolution", map_resolution, (float)0.05);

    nh_.param("/global_planner_node/goal_weight", goal_weight, (float)1.5);
    nh_.param("/global_planner_node/cost_weight", cost_weight, (float)0.2);
    nh_.param("/global_planner_node/lof_distance", lof_distance, (float)0.2);
    nh_.param("/global_planner_node/occ_threshold", occ_threshold, (float)99);
    nh_.param("/global_planner_node/traj_dxy_max", traj_dxy_max, (float)10);
    nh_.param("/global_planner_node/traj_pos_tol", traj_pos_tol, (float)10);
    nh_.param("/global_planner_node/traj_yaw_tol", traj_yaw_tol, (float)0.2);
    nh_.param("/global_planner_node/world_frame", world_frame, (string) "/map");
    nh_.param("/global_planner_node/robot_base_frame", robot_base_frame, (string) "/base_link");

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner Node Configuration:");
    //ROS_INFO_COND(showConfig, PRINTF_GREEN "\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] ", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
    //ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Map: resol.= [%.2f]", map_resolution);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* with optim.: goal_weight = [%.2f]", goal_weight);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Occupied threshold = %.0f", occ_threshold);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Cost weight = [%.2f]", cost_weight);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Line of sight restriction = [%.2f]", lof_distance);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t World frame: %s, Robot base frame: %s", world_frame.c_str(), robot_base_frame.c_str());

    //Configure markers
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "global_path";
    marker.lifetime = ros::Duration(200);
    marker.type = RVizMarker::ARROW;
    marker.action = RVizMarker::DELETEALL;
    marker.pose.position.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.4;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
}
bool GlobalPlanner::replanningSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep)
{
    rep.message = "Calculating New Global Path";
    //Load the more recent map and calculate a new path
    gbPlanner.getMap(&globalCostMap);

    rep.success = calculatePath();
    return true;
}
void GlobalPlanner::dynReconfCb(theta_star_2d::globalPlannerConfig &config, uint32_t level)
{
    this->cost_weight = config.cost_weight;
    this->lof_distance = config.lof_distance;
    this->goal_weight = config.goal_weight;
    this->occ_threshold = config.occ_threshold;
    gbPlanner.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: Dynamic reconfigure requested");
}
void GlobalPlanner::globalCostMapCb(const nav_msgs::OccupancyGrid::ConstPtr &fp)
{
    globalCostMap = *fp;
    gCmReceived = true;
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: Global map received");
    if (!mapParamsConfigured)
    {
        map_resolution = round(100 * (fp->info.resolution)) / 100;
        ws_x_max = fp->info.width * map_resolution;
        ws_y_max = fp->info.height * map_resolution;
        ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: ws_x_max,ws_y_max, map_resolution: [%.2f, %.2f, %.2f]", ws_x_max, ws_y_max, map_resolution);
        gbPlanner.loadMapParams(ws_x_max, ws_y_max, map_resolution);
        mapParamsConfigured = true;
    }
}
void GlobalPlanner::goalCb(const geometry_msgs::PoseStamped::ConstPtr &goalMsg)
{
    //We neeed the stamped goal with the rotation to put it in the trajectory message
    goalPoseStamped = *goalMsg;
    //We need a vector3 because ThetaStar wants a Vector3 data type
    goal.vector.x = goalMsg->pose.position.x;
    goal.vector.y = goalMsg->pose.position.y;
    goal.header = goalMsg->header;

    globalGoalReceived = true;

    ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: Goal received");
}
void GlobalPlanner::plan()
{

    if (gCmReceived)
    {
        //TODO: Control maps timeout when using non static maps, like global cosmtap with dynamics obstacles refreshing at low rate
        gbPlanner.getMap(&globalCostMap);
        //Set the flag to false in order to not pass the same map again to algorithm if no new map received
        gCmReceived = false;
    }
    if (globalGoalReceived && mapParamsConfigured)
    {
        ROS_INFO_COND(debug, PRINTF_MAGENTA "Goal received");
        calculatePath();
        //Reset flag and wait for a new goal
        globalGoalReceived = false;
    }
}
bool GlobalPlanner::calculatePath()
{
    //It seems that you can get a goal and no map and try to get a path but setGoal and setStart will check if the points are valid
    //so if there is no map received it won't calculate a path
    if (setGoal() && setStart())
    {
        ROS_INFO_COND(debug, PRINTF_MAGENTA "Goal and start successfull set");
        // Path calculation
        number_of_points = gbPlanner.computePath();

        if (number_of_points > 0)
        {
            ROS_INFO_COND(debug, PRINTF_MAGENTA "Publishing trajectory");
            publishTrajectory();
            return true;
        }
    }
    else
    {
        ROS_ERROR("Global goal or start point occupied ");
    }
    return false;
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

    trajectory.points.clear();
    trajectory.header.stamp = ros::Time::now();

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

    marker.action = RVizMarker::DELETEALL;
    //First send a message with a DELETE marker to remove previous markers
    markerTraj.markers.empty();
    markerTraj.markers.push_back(marker);
    vis_trj_pub.publish(markerTraj);
    markerTraj.markers.empty();
    //Now change action to ADD (normal work)
    marker.action = RVizMarker::ADD;
    marker.header.stamp = ros::Time::now();

    for (int i = 0; i < trajectory.points.size(); i++)
    {
        marker.pose.position.x = trajectory.points[i].transforms[0].translation.x;
        marker.pose.position.y = trajectory.points[i].transforms[0].translation.y;
        marker.pose.orientation = trajectory.points[i].transforms[0].rotation;
        marker.id = rand();
        markerTraj.markers.push_back(marker);
    }
    vis_trj_pub.publish(markerTraj);
}
bool GlobalPlanner::setGoal()
{
    bool ret;
    if (gbPlanner.setValidFinalPosition(goal.vector))
    {
        ret = true;
    }
    else
    {
        ROS_ERROR("Global Planner: Failed to set final global position");
        ret = false;
    }
    return ret;
}
bool GlobalPlanner::setStart()
{
    bool ret;
    geometry_msgs::Vector3Stamped start;
    start.vector.x = getRobotPose().transform.translation.x;
    start.vector.y = getRobotPose().transform.translation.y;

    if (gbPlanner.setValidInitialPosition(start.vector))
    {
        ret = true;
    }
    else
    {
        ROS_ERROR("Global Planner: Failed to set initial global position");
        ret = false;
    }
    return ret;
}
} // namespace PathPlanners