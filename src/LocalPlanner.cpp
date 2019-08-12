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
}
void LocalPlanner::configParams()
{
    //Flags for flow control
    localCostMapReceived = false;
    globalTrajReceived = false;
    localGoalReached = true;
    globalGoalReached = false;
    mapGeometryConfigured = false;
    //nh_.param("/costmap_2d_local/costmap/width", ws_x_max, (float)0);
    //nh_.param("/costmap_2d_local/costmap/height", ws_y_max, (float)0);
    //nh_.param("/costmap_2d_local/costmap/resolution", map_resolution, (float)0.05);

    //nh_.param("/local_planner_node/ws_x_max", ws_x_max, (float)6);
    //nh_.param("/local_planner_node/ws_y_max", ws_y_max, (float)6);
    //nh_.param("/local_planner_node/map_resolution", map_resolution, (float)0.05);

    nh_.param("/local_planner_node/goal_weight", goal_weight, (float)1.5);
    nh_.param("/local_planner_node/cost_weight", cost_weight, (float)0.2);
    nh_.param("/local_planner_node/lof_distance", lof_distance, (float)1.5);
    nh_.param("/local_planner_node/occ_threshold", occ_threshold, (float)99);

    nh_.param("/local_planner_node/traj_dxy_max", traj_dxy_max, (float)10);
    nh_.param("/local_planner_node/traj_pos_tol", traj_pos_tol, (float)10);
    nh_.param("/local_planner_node/traj_yaw_tol", traj_yaw_tol, (float)0.2);

    nh_.param("/local_planner_node/world_frame", world_frame, (string) "/map");
    nh_.param("/local_planner_node/robot_base_frame", robot_base_frame, (string) "/base_link");
    nh_.param("/local_planner_node/local_costmap_infl_x", localCostMapInflationX, (float)1);
    nh_.param("/local_planner_node/local_costmap_infl_y", localCostMapInflationY, (float)1);
    nh_.param("/local_planner_node/border_space", border_space, (float)1);

    nh_.param("/local_planner_node/debug", debug, (bool)0);
    nh_.param("/local_planner_node/show_config", showConfig, (bool)0);

    //ws_x_max += 2 * localCostMapInflationX;
    //ws_y_max += 2 * localCostMapInflationY;
    //ws_x_min = 0;
    //ws_y_min = 0;

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Local Planner Node Configuration:\n");
    //ROS_INFO_COND(showConfig, PRINTF_GREEN "\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f]", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
    //ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Local Costmap Origin: [%.2f, %.2f]", local_costmap_center.x, local_costmap_center.y);
    //ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Map: resol.= [%.2f]", map_resolution);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* with optim.: goal_weight = [%.2f]", goal_weight);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* modified: cost_weight = [%.2f], lof_distance = [%.2f]", cost_weight, lof_distance);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* occ_threshold = %f", occ_threshold);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Local Costmap Extra Borders = [%.2f, %.2f]", localCostMapInflationX, localCostMapInflationY);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Free space around local goal inside borders = [%.2f]", border_space);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Robot base frame: %s, World frame: %s", robot_base_frame.c_str(), world_frame.c_str());

    markerTraj.header.frame_id = world_frame;
    markerTraj.header.stamp = ros::Time();
    markerTraj.ns = "local_path";
    markerTraj.id = rand();
    markerTraj.type = RVizMarker::SPHERE_LIST;
    markerTraj.action = RVizMarker::ADD;
    markerTraj.pose.orientation.w = 1.0;
    markerTraj.lifetime = ros::Duration(0.5);
    markerTraj.scale.x = 0.2;
    markerTraj.scale.y = 0.2;
    markerTraj.pose.position.z = 1.0;
    markerTraj.color.a = 1.0;
    markerTraj.color.r = 1.0;
    markerTraj.color.g = 0.0;
    markerTraj.color.b = 0.0;

    localGoal.x = 0;
    localGoal.y = 0;

    occ.data = false;
    is_running.data = true;
    impossible_calculate.data = false;

    impossibles = 0;
    number_of_points = 0;
    startOk = false;
}
void LocalPlanner::configTheta()
{
    string node_name = "local_planner_node";
    //lcPlanner.init(node_name, world_frame, ws_x_max, ws_y_max, ws_x_min, ws_y_min, map_resolution, goal_weight, cost_weight, lof_distance, occ_threshold, &nh_);
    lcPlanner.initAuto(node_name, world_frame,goal_weight, cost_weight, lof_distance, &nh_);
    lcPlanner.setTimeOut(20);
    lcPlanner.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Theta Star Configured");
}
void LocalPlanner::configTopics()
{

    string topicPath;
    //First Subscribers
    nh_.param("/local_planner_node/local_costmap_topic", topicPath, (string) "/costmap_2d_local/costmap/costmap");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Local Costmap Topic: %s", topicPath.c_str());
    local_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>(topicPath, 1, &LocalPlanner::localCostMapCb, this);

    nh_.param("/local_planner_node/goal_reached_topic", topicPath, (string) "/trajectory_tracker/local_goal_reached");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Goal Reached Flag Topic: %s", topicPath.c_str());
    goal_reached_sub = nh_.subscribe<std_msgs::Bool>(topicPath, 1, &LocalPlanner::goalReachedCb, this);

    nh_.param("/local_planner_node/global_goal_topic", topicPath, (string) "/move_base_simple/goal");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Global Goal Topic: %s", topicPath.c_str());
    global_goal_sub = nh_.subscribe<geometry_msgs::PoseStamped>(topicPath, 1, &LocalPlanner::globalGoalCb, this);

    nh_.param("/local_planner_node/global_traj_topic", topicPath, (string) "/trajectory_tracker/input_trajectory");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Global Trajectory Topic: %s", topicPath.c_str());
    global_trj_sub = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 1, &LocalPlanner::globalTrjCb, this);
    //Now publishers

    nh_.param("/local_planner_node/local_trajectory_ouput_topic", topicPath, (string) "/trajectory_tracker/local_input_trajectory");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Trajectory output topic: %s", topicPath.c_str());
    trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topicPath, 0);

    nh_.param("/local_planner_node/local_trajectory_markers_topic", topicPath, (string) "/local_planner_node/visualization_marker_trajectory");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Visualization marker traj. output topic: %s", topicPath.c_str());
    vis_marker_traj_pub = nh_.advertise<visualization_msgs::Marker>(topicPath, 0);

    nh_.param("/local_planner_node/global_goal_topic", topicPath, (string) "/move_base_simple/goal");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Global goal topic topic: %s", topicPath.c_str());
    global_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>(topicPath, 0);

    nh_.param("/local_planner_node/local_planning_time_topic", topicPath, (string) "/local_planning_time");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Local Planning time output topic: %s", topicPath.c_str());
    local_planning_time = nh_.advertise<std_msgs::Int32>(topicPath, 0);

    nh_.param("/local_planner_node/local_costmap_inflated_topic", topicPath, (string) "/local_costmap_inflated");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Local Costmap Inflated Borders output topic: %s", topicPath.c_str());
    inf_costmap_pub = nh_.advertise<nav_msgs::OccupancyGrid>(topicPath, 0);

    //Flags publishers

    nh_.param("/local_planner_node/running_flag_topic", topicPath, (string) "/local_planner_node/running");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Running state output topic: %s", topicPath.c_str());
    running_state_pub = nh_.advertise<std_msgs::Bool>(topicPath, 0);

    nh_.param("/local_planner_node/occupied_goal_flag_topic", topicPath, (string) "/trajectory_tracker/local_goal_occupied");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Local goal occupied output topic: %s", topicPath.c_str());
    occ_goal_pub = nh_.advertise<std_msgs::Bool>(topicPath, 0);

    nh_.param("/local_planner_node/impossible_to_find_solution_topic_flag", topicPath, (string) "/trajectory_tracker/impossible_to_find");
    ROS_INFO_COND(showConfig, PRINTF_CYAN "\t Local Planner: Impossible to find solution output topic: %s", topicPath.c_str());
    impossible_to_find_sol_pub = nh_.advertise<std_msgs::Bool>(topicPath, 0);
}
//Calbacks and publication functions
void LocalPlanner::localCostMapCb(const nav_msgs::OccupancyGrid::ConstPtr &lcp)
{
    localCostMap = *lcp;
    localCostMapReceived = true;
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Received local costmap");
    //First time the map is received, configure the geometric params
    if (!mapGeometryConfigured)
    {
       
        map_resolution = round(100* (lcp->info.resolution))/100;//Because sometimes the map server shows not exact long numbers as 0.0500003212
        ws_x_max = lcp->info.width *map_resolution+ 2 * localCostMapInflationX;
        ws_y_max = lcp->info.height*map_resolution + 2 * localCostMapInflationY;
        local_costmap_center.x = ws_x_max / 2;
        local_costmap_center.y = ws_y_max / 2;
        ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: ws_x_max,ws_y_max, map_resolution: [%.2f, %.2f, %.2f]", ws_x_max, ws_y_max, map_resolution);
        lcPlanner.loadMapParams(ws_x_max,ws_y_max,map_resolution);
        mapGeometryConfigured = true;
        ROS_INFO_COND(showConfig, PRINTF_GREEN "\t WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f]", ws_x_min, ws_x_max, ws_y_min, ws_y_max);
        ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Local Costmap Origin: [%.2f, %.2f]", local_costmap_center.x, local_costmap_center.y);
        ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Map: resol.= [%.2f]", map_resolution);
    }
}
//Global Input trajectory
void LocalPlanner::globalTrjCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &traj)
{
    globalTrajectory = *traj;
    globalTrajArrLen = globalTrajectory.points.size();
    startIter = 1;
    globalTrajectory.header.frame_id = world_frame;
    globalTrajReceived = true;
    globalGoalReached = false;
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Received global trajectory");
}
//
void LocalPlanner::globalGoalCb(const geometry_msgs::PoseStamped::ConstPtr &goal)
{
    globalGoalStamped = *goal;
    globalTrajReceived = false;
    globalGoalReached = false;
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Received Global goal");
}

//Used to know when to stop calculating local trajectories
void LocalPlanner::goalReachedCb(const std_msgs::Bool::ConstPtr &data)
{
    globalGoalReached = true;
    globalTrajReceived = false;
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Goal reached message received!");
}

void LocalPlanner::dynRecCb(theta_star_2d::localPlannerConfig &config, uint32_t level)
{
    this->cost_weight = config.cost_weight;
    this->lof_distance = config.lof_distance;
    this->occ_threshold = config.occ_threshold;
    this->goal_weight = config.goal_weight;

    lcPlanner.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);
    ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Dynamic reconfiguration required");
}
void LocalPlanner::plan()
{
    ftime(&startT);

    running_state_pub.publish(is_running);
    number_of_points = 0; //Reset variable

    if (globalTrajReceived && localCostMapReceived)
    {
        //ROS_INFO("Local: Global Traj Received");
        ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Global trj received and local costmap received");
        localCostMapReceived = false;
        if (!lcPlanner.setValidInitialPosition(local_costmap_center))
        {
            if (lcPlanner.searchInitialPosition2d(0.5))
            {
                startOk = true;
            }
            else
            {
                ROS_INFO("Local:Couldn't find a free point near start point");
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

            inflateCostMap();

            freeLocalGoal();

            inf_costmap_pub.publish(localCostMapInflated);

            lcPlanner.getMap(&localCostMapInflated);

            if (!lcPlanner.setValidFinalPosition(localGoal))
            {

                if (lcPlanner.searchFinalPosition2d(0.2)) //Estaba a 0.2 antes(en la demo de portugal)
                {
                    ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Computing Local Path(1)");
                    number_of_points = lcPlanner.computePath();
                }
                else
                {
                    ROS_INFO("Local: Couldn't find a free point near local goal ");
                    globalTrajReceived = false;
                    occ.data = true;
                    occ_goal_pub.publish(occ);
                    global_goal_pub.publish(globalGoalStamped);
                }
            }
            else
            {
                if (occ.data)
                {
                    //ROS_INFO("Local: Local goal desocuupied");
                    occ.data = false;
                    occ_goal_pub.publish(occ);
                }
                ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Computing Local Path(2)");
                number_of_points = lcPlanner.computePath();
            }

            if (number_of_points > 0 && !occ.data)
            {
                ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Building and publishing local Path");
                buildAndPubTrayectory();
                ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Local Path build and published");
                publishTrajMarker();
                ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Published trajectory markers");
                startOk = false;

                if (impossibles > 0)
                {
                    //ROS_INFO("Local: Impossible reseted");

                    impossible_calculate.data = false;
                    impossible_to_find_sol_pub.publish(impossible_calculate);
                    impossibles = 0;
                }
            }
            else
            {
                impossibles++;
                //ROS_INFO("Local: +1 impossible");
                if (impossibles == 10)
                {

                    impossible_calculate.data = true;
                    impossible_to_find_sol_pub.publish(impossible_calculate);
                    global_goal_pub.publish(globalGoalStamped);
                    globalTrajReceived = false;
                    //startOk = false;
                    ROS_WARN("Requesting new global path to same global goal");
                    //Tambien publicar
                    //impossibles=0;
                }
            }
        }
    }

    ftime(&finishT);
    seconds = finishT.time - startT.time - 1;
    milliseconds = (1000 - startT.millitm) + finishT.millitm;
    time_spent_msg.data = (milliseconds + seconds * 1000);
    //if (debug)
    //    showTime(PRINTF_YELLOW "Time spent", startT, finishT);

    local_planning_time.publish(time_spent_msg);
}

geometry_msgs::Vector3 LocalPlanner::calculateLocalGoal()
{

    geometry_msgs::Vector3Stamped C;

    trajectory_msgs::MultiDOFJointTrajectory globalTrajBLFrame = globalTrajectory;
    //First we transform the trajectory published by global planner to base_link frame

    geometry_msgs::TransformStamped tr = getTransformFromMapToBaseLink();

    globalTrajBLFrame.header.frame_id = robot_base_frame;

    for (int i = 0; i < globalTrajArrLen; i++)
    {
        globalTrajBLFrame.points[i].transforms[0].translation.x -= tr.transform.translation.x;
        globalTrajBLFrame.points[i].transforms[0].translation.y -= tr.transform.translation.y;
    }

    //Ya esta referida al base_link. Ahora la recorro desde i=1(porque i=0 es siempre la pos del base_link que al pasarla al sistema base_link sera (0,0))
    for (int i = startIter; i < globalTrajArrLen; i++)
    {
        C.vector.x = globalTrajBLFrame.points[i].transforms[0].translation.x + (ws_x_max) / 2;
        C.vector.y = globalTrajBLFrame.points[i].transforms[0].translation.y + (ws_y_max) / 2;

        if (fabs(globalTrajBLFrame.points[i].transforms[0].translation.x) > (ws_x_max / 2 - localCostMapInflationX) ||
            fabs(globalTrajBLFrame.points[i].transforms[0].translation.y) > (ws_y_max / 2 - localCostMapInflationY) || i == globalTrajArrLen - 1)
        {
            startIter = i;
            break;
        }
    }
    //ROS_INFO_THROTTLE(1, PRINTF_BLUE "[%.2f, %.2f]", C.vector.x, C.vector.y);
    C.vector.x = floor(C.vector.x * 10 + 10 * map_resolution) / 10;
    C.vector.y = floor(C.vector.y * 10 + 10 * map_resolution) / 10;
    if (C.vector.x == 0)
        C.vector.x += 0.05;
    if (C.vector.y == 0)
        C.vector.y += 0.05;

    return C.vector;
}

geometry_msgs::TransformStamped LocalPlanner::getTransformFromMapToBaseLink()
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
    markerTraj.points.clear();
    geometry_msgs::Point p;
    for (int i = 0; i < localTrajArrLen; i++)
    {
        p.x = localTrajectory.points[i].transforms[0].translation.x;
        p.y = localTrajectory.points[i].transforms[0].translation.y;
        markerTraj.points.push_back(p);
    }
    vis_marker_traj_pub.publish(markerTraj);
}

void LocalPlanner::buildAndPubTrayectory()
{
    trajectory_msgs::MultiDOFJointTrajectoryPoint goal_temp;
    geometry_msgs::Transform temp1;
    //La trayectoria se obtiene en el frame del local costmap, que tiene la misma orientacion que el map pero esta centrado en el base_link
    localTrajectory.points.clear();
    lcPlanner.getTrajectoryYawInAdvance(localTrajectory, getTransformFromMapToBaseLink().transform);

    localTrajArrLen = localTrajectory.points.size();

    for (int i = 0; i < localTrajArrLen; i++)
    {
        localTrajectory.points[i].transforms[0].translation.x += localCostMapInflated.info.origin.position.x;
        localTrajectory.points[i].transforms[0].translation.y += localCostMapInflated.info.origin.position.y;
    }
    localGoal.x += localCostMapInflated.info.origin.position.x;
    localGoal.y += localCostMapInflated.info.origin.position.y;

    temp1.translation.x = localGoal.x;
    temp1.translation.y = localGoal.y;
    //TODO: PONER LA OPRIENTATION DEL WAYPOINT SIGUIENTE AL LOCAL GOAL
    temp1.rotation.w = 1;

    goal_temp.transforms.resize(1, temp1);

    localTrajectory.points.push_back(goal_temp);
    localTrajArrLen = localTrajectory.points.size();

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
    // TODO: Lo mas eficiente no es esto, sino comprobar primero los casos mas probables)

    //Para todos los bucles siempre es igual
    // ! i: Numero de fila
    // ! j: columna
    if (localGoal.y > localCostMapInflated.info.height * map_resolution - localCostMapInflationY)
    {
        //Esquina 1 o 3 o borde 2
        if (localGoal.x < localCostMapInflationX) //1
        {
            for (int i = localCostMapInflated.info.height - 2 * localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i++)
                for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;

            for (int i = localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height; i++)
                for (int j = 0; j < 2 * localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else if (localGoal.x > localCostMapInflated.info.width * map_resolution - localCostMapInflationX) //5
        {
            for (int i = localCostMapInflated.info.height - 2 * localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i++)
                for (int j = localCostMapInflated.info.width - localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;

            for (int i = localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height; i++)
                for (int j = localCostMapInflated.info.width - 2 * localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else //Borde 2
        {

            for (int i = localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height; i++)
            {
                for (int j = (localGoal.x - border_space) / map_resolution; j < (localGoal.x + border_space) / map_resolution; j++)
                {
                    localCostMapInflated.data[i * localCostMapInflated.info.width + j] = 0;
                }
            }
        }
    }
    else if (localGoal.y < localCostMapInflationY)
    {                                                                                                //Esquina 3 o 1 o borde 6
        if (localGoal.x > localCostMapInflated.info.width * map_resolution - localCostMapInflationX) //3
        {
            for (int i = 0; i < localCostMapInflationY / map_resolution; i++)
                for (int j = localCostMapInflated.info.width - 2 * localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            for (int i = localCostMapInflationY / map_resolution; i < 2 * localCostMapInflationY / map_resolution; i++)
                for (int j = localCostMapInflated.info.width - localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else if (localGoal.x < localCostMapInflationX) //1
        {
            for (int i = 0; i < localCostMapInflationY / map_resolution; i++) //Filas
                for (int j = 0; j < 2 * localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            for (int i = localCostMapInflationY / map_resolution; i < 2 * localCostMapInflationY / map_resolution; i++)
                for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else //Borde 6
        {
            for (int i = 0; i < localCostMapInflationY / map_resolution; i++)
            {
                for (int j = (localGoal.x - border_space) / map_resolution; j < (localGoal.x + border_space) / map_resolution; j++)
                {
                    localCostMapInflated.data[i * localCostMapInflated.info.width + j] = 0;
                }
            }
        }
    } //Si hemos llegado hasta aqui sabemos que la y esta dentro del costmap original
    else if (localGoal.x < localCostMapInflationX)
    { //Borde 8
        //Se recorren las filas desde la primera hasta la ultima y se ponen a 0 los N primeros valores de cada fila(numero de columnas infladas)
        for (int i = (localGoal.y - border_space) / map_resolution; i < (localGoal.y + border_space) / map_resolution; i++)
        {
            for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
                localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
    }
    else if (localGoal.x > localCostMap.info.width * map_resolution + localCostMapInflationX)
    { //Borde 4
        //Se recorren las filas desde la primera hasta la ultima y se ponen a 0 los N ultimos valores de cada fila (numero de columnas infladas)
        for (int i = (localGoal.y - border_space) / map_resolution; i < (localGoal.y + border_space) / map_resolution; i++)
        {
            for (int j = localCostMapInflated.info.width - localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
            {
                localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            }
        }
    }
    else
    {
        ROS_WARN_THROTTLE(1, "No he entrado en ninguna zona");
    }
}

} // namespace PathPlanners