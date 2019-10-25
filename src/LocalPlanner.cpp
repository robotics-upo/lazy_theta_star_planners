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
void LocalPlanner::clearMarkers()
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
void LocalPlanner::configServices()
{
    costmap_clean_srv = nh_.serviceClient<std_srvs::Trigger>("/custom_costmap_node/reset_costmap");
    
    execute_path_srv_ptr.reset(new ExecutePathServer(nh_, "Execute_Plan", false));
    execute_path_srv_ptr->registerGoalCallback(boost::bind(&LocalPlanner::executePathGoalServerCB, this));
    execute_path_srv_ptr->registerPreemptCallback(boost::bind(&LocalPlanner::executePathPreemptCB, this));
    execute_path_srv_ptr->start();

    navigate_client_ptr.reset(new NavigateClient("Navigation", true));

    //?navigate_client_ptr->waitForServer();
}
void LocalPlanner::configParams()
{
    //Flags for flow control
    localCostMapReceived = false;
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

    //Line strip marker use member points, only scale.x is used to control linea width
    lineMarker.header.frame_id = world_frame;
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.id = rand();
    lineMarker.ns = "local_path";
    lineMarker.lifetime = ros::Duration(500);
    lineMarker.type = RVizMarker::LINE_STRIP;
    lineMarker.action = RVizMarker::ADD;
    lineMarker.pose.orientation.w = 1;
    lineMarker.color.r = 1.0;
    lineMarker.color.g = 0.0;
    lineMarker.color.b = 0.0;
    lineMarker.color.a = 1.0;
    lineMarker.scale.x = 0.1;

    waypointsMarker.header.frame_id = world_frame;
    waypointsMarker.header.stamp = ros::Time::now();
    waypointsMarker.ns = "local_path";
    waypointsMarker.id = lineMarker.id + 12;
    waypointsMarker.lifetime = ros::Duration(500);
    waypointsMarker.type = RVizMarker::POINTS;
    waypointsMarker.action = RVizMarker::ADD;
    waypointsMarker.pose.orientation.w = 1;
    waypointsMarker.color.r = 0.0;
    waypointsMarker.color.g = 0.0;
    waypointsMarker.color.b = 1.0;
    waypointsMarker.color.a = 1.0;
    waypointsMarker.scale.x = 0.15;
    waypointsMarker.scale.y = 0.15;
    waypointsMarker.scale.z = 0.4;

    is_running.data = true;

    impossibleCnt = 0;
    occGoalCnt = 0;
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
void LocalPlanner::executePathGoalServerCB() // Note: "Action" is not appended to exe here
{
    ROS_INFO_COND(debug, "Local Planner Goal received in action server mode");
    upo_actions::ExecutePathGoalConstPtr path_shared_ptr;
    path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();

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
    nav_goal.global_goal.position.y = globalTrajectory.points.at(--siz).transforms[0].translation.y;
    navigate_client_ptr->sendGoal(nav_goal);
}
void LocalPlanner::configTheta()
{
    string node_name = "local_planner_node";
    lcPlanner.initAuto(node_name, world_frame, goal_weight, cost_weight, lof_distance, &nh_);
    lcPlanner.setTimeOut(1);
    lcPlanner.setTrajectoryParams(traj_dxy_max, traj_pos_tol, traj_yaw_tol);
    //ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Theta Star Configured");
}
void LocalPlanner::configTopics()
{
    local_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>("/custom_costmap_node/costmap/costmap", 1, &LocalPlanner::localCostMapCb, this);
    trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/trajectory_tracker/local_input_trajectory", 0);
    visMarkersPublisher = nh_.advertise<visualization_msgs::Marker>("/local_planner_node/markers", 30);
    running_state_pub = nh_.advertise<std_msgs::Bool>("/local_planner_node/running", 0);
}
//Calbacks and publication functions
void LocalPlanner::localCostMapCb(const nav_msgs::OccupancyGrid::ConstPtr &lcp)
{
    localCostMap = *lcp;
    localCostMapReceived = true;

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
void LocalPlanner::plan()
{
    running_state_pub.publish(is_running);
    number_of_points = 0; //Reset variable

    if (!execute_path_srv_ptr->isActive())
    {
        clearMarkers();
        return;
    }
    else if (navigate_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        clearMarkers();
        action_result.arrived = true;
        execute_path_srv_ptr->setSucceeded(action_result);
        ROS_ERROR("LocalPlanner: Goal Succed");
        return;
    }
    else if(navigate_client_ptr->getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO("Goal aborted by path tracker");
    }else if(navigate_client_ptr->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
        ROS_INFO("Goal preempted by path tracker");
    }

    ftime(&startT);
    if (localCostMapReceived && doPlan)
    {
        ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Global trj received and local costmap received");
        localCostMapReceived = false;
        lcPlanner.getMap(&localCostMapInflated);

        if (lcPlanner.setValidInitialPosition(local_costmap_center) || lcPlanner.searchInitialPosition2d(0.3))
        {
            calculateLocalGoal();
            inflateCostMap();

            if (lcPlanner.setValidFinalPosition(localGoal) || lcPlanner.searchFinalPosition2d(0.3))
            {
                ROS_INFO_COND(debug, PRINTF_YELLOW "Local Planner: Computing Local Path(2)");
                number_of_points = lcPlanner.computePath();
                occGoalCnt = 0;
                
                if (number_of_points > 0)
                {
                    buildAndPubTrayectory();
                    planningStatus.data = "OK";
                    if (impossibleCnt > 0) //If previously the local planner couldn t find solution, reset
                        impossibleCnt = 0;
                }
                else if (number_of_points == 0)//!Esto es lo que devuelve el algoritmo cuando NO HAY SOLUCION
                {
                    impossibleCnt++;
                    //ROS_INFO("Local: +1 impossible");
                    if (impossibleCnt > 2)
                    {
                        navigate_client_ptr->cancelGoal();

                        planningStatus.data = "Requesting new global path, navigation cancelled";
                        execute_path_srv_ptr->setAborted();
                        impossibleCnt=0;
                    }
                }
            }
            else if (occGoalCnt > 2) //!Caso GOAL OCUPADO
            {//If it cant find a free position near local goal, it means that there is something there.

                ROS_INFO_COND(debug, PRINTF_YELLOW "Pausing planning, final position busy");
                planningStatus.data = "Final position Busy, Cancelling goal";
                //TODO What to tell to the path tracker
                navigate_client_ptr->cancelGoal();
                execute_path_srv_ptr->setAborted();
                //In order to resume planning, someone must call the pause/resume planning Service that will change the flag to true
                occGoalCnt = 0;
            }
            else
            {
                occGoalCnt++;
            }
        }
        else
        {
            ROS_INFO("Local:Couldn't find a free point near start point, trying to clean the local costmap");
            std_srvs::Trigger trg;
            costmap_clean_srv.call(trg);
        }
    }
    ftime(&finishT);

    seconds = finishT.time - startT.time - 1;
    milliseconds = (1000 - startT.millitm) + finishT.millitm;

    if (execute_path_srv_ptr->isActive())
        publishExecutePathFeedback();
}
void LocalPlanner::publishExecutePathFeedback()
{

    if (planningStatus.data.find("OK") > 0)
    {
        planningRate.data = 1000 / milliseconds;
    }
    else
    {
        planningRate.data = 0;
    }

    exec_path_fb.global_waypoint = waypointGoingTo;
    exec_path_fb.planning_rate = planningRate;
    exec_path_fb.status = planningStatus;
    execute_path_srv_ptr->publishFeedback(exec_path_fb);
}
void LocalPlanner::calculateLocalGoal()
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

    localGoal = C.vector;
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
    //!This is done to clear out the previous markers
    waypointsMarker.action = RVizMarker::DELETEALL;
    lineMarker.action = RVizMarker::DELETEALL;

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);

    lineMarker.points.clear();
    waypointsMarker.points.clear();

    lineMarker.action = RVizMarker::ADD;
    waypointsMarker.action = RVizMarker::ADD;

    lineMarker.header.stamp = ros::Time::now();
    waypointsMarker.header.stamp = ros::Time::now();

    geometry_msgs::Point p;

    for (int i = 0; i < localTrajectory.points.size(); i++)
    {
        p.x = localTrajectory.points[i].transforms[0].translation.x;
        p.y = localTrajectory.points[i].transforms[0].translation.y;

        lineMarker.points.push_back(p);
        waypointsMarker.points.push_back(p);
    }

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);
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
    publishTrajMarker();
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

    freeLocalGoal();
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