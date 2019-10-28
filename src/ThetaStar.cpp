/*
 * Copyright 2016 Ricardo Ragel de la Torre, GRVC, Univ. of Seville, Spain
 *
 * Resume: 	ThetaStar Class definitions.
 * 			Motion Path Planner based on the Theta Star algoritm: 
 * 			"Weighted Lazy Theta Star with Optimization" and different
 * 			methods to get a colission-free path [x,y,z] or 
 * 			trajectory [xyz, yaw, time].
 */

#include <theta_star/ThetaStar.hpp>

namespace PathPlanners
{

// Uncomment to printf the explored nodes number
//#define PRINT_EXPLORED_NODES_NUMBER
// Uncomment to get the explored nodes (at time or slowly step_by_step)
#define SEND_EXPLORED_NODES_MARKERS
//#define STEP_BY_STEP
// Uncomment to get non-LineOfSight visual markers
#define SEND_NO_LOFS_NODES_MARKERS
// Uncomment to printf if setVertex() fails at no-LofS
#define PRINT_SETVERTEX_FAILS
#define TESTING_FUNCT
//*****************************************************************
// 				ThetaStar Algorithm Class Definitions
//*****************************************************************
// Default constructor
ThetaStar::ThetaStar()
{
}

// Constructor with arguments
ThetaStar::ThetaStar(string plannerName, string frame_id,
                     float ws_x_max_, float ws_y_max_, float ws_x_min_, float ws_y_min_,
                     float step_, float goal_weight_, float cost_weight_, float lof_distance_, int occ_threshold_, ros::NodeHandlePtr n)
{
    // Call to initialization
    //ROS_WARN("\t 2WorkSpace: X:[%.2f, %.2f], Y:[%.2f, %.2f] \n", ws_x_min_, ws_x_max_, ws_y_min_, ws_y_max_);
    init(plannerName, frame_id, ws_x_max_, ws_y_max_, ws_x_min_, ws_y_min_, step_, goal_weight_, cost_weight_, lof_distance_, occ_threshold_, n);
}
ThetaStar::ThetaStar(string plannerName, string frame_id, float goal_weight_, float cost_weight_, float lof_distance_, ros::NodeHandlePtr n)
{

    mapParamsConfigured = false;
    initAuto(plannerName, frame_id, goal_weight_, cost_weight_, lof_distance_, n);
}
void ThetaStar::setDynParams(float goal_weight_, float cost_weight_, float lof_distance_, int occ_threshold_)
{
    goal_weight = goal_weight_;
    cost_weight = cost_weight_;
    lof_distance = lof_distance_;
    occ_threshold = occ_threshold_;
}
// Initialization: creates the occupancy matrix (discrete nodes) from the bounding box sizes, resolution, inflation and optimization arguments
/*void ThetaStar::initAuto(string plannerName, string frame_id, float goal_weight_, float cost_weight_, float lof_distance_, ros::NodeHandle *n)
{
    nh = n;

    goal_weight = goal_weight_;
    cost_weight = cost_weight_;
    lof_distance = lof_distance_;

    //Configure ws_y_min and ws_x_min to zero by default
    ws_x_min = 0;
    ws_y_min = 0;

    //Finally configure markers
    configureMarkers(plannerName, frame_id, step);
    //The geometric params will be passed to thetastar once the planners receive the first map
}*/
// Initialization: creates the occupancy matrix (discrete nodes) from the bounding box sizes, resolution, inflation and optimization arguments
void ThetaStar::initAuto(string plannerName, string frame_id, float goal_weight_, float cost_weight_, float lof_distance_, ros::NodeHandlePtr n)
{
    nh = n;
    
    goal_weight = goal_weight_;
    cost_weight = cost_weight_;
    lof_distance = lof_distance_;

    //Configure ws_y_min and ws_x_min to zero by default
    ws_x_min = 0;
    ws_y_min = 0;

    //Finally configure markers
    configureMarkers(plannerName, frame_id, step);
    //The geometric params will be passed to thetastar once the planners receive the first map
}
void ThetaStar::loadMapParams(float ws_x_max_, float ws_y_max_, float map_resolution_)
{
    disc_initial = NULL;
    disc_final = NULL;

    // Init asymetric and inflated occupancy matrix
    ws_x_max = round(ws_x_max_ / map_resolution_);
    ws_y_max = round(ws_y_max_ / map_resolution_);
    step = map_resolution_;
    step_inv = 1.0 / map_resolution_;

    Lx = ws_x_max;
    Ly = ws_y_max;

    Lx_inv = 1.0 / Lx;
    Ly_inv = 1.0 / Ly;
    matrix_size = Lx * Ly;

    printf("1. ThetaStar: Occupancy Matrix has %d nodes [%d MB]\n", matrix_size, (int)(matrix_size * sizeof(ThetaStarNode)) / (1024 * 1024));
    discrete_world.resize(matrix_size);
    mapParamsConfigured = true;
}
void ThetaStar::init(string plannerName, string frame_id,
                     float ws_x_max_, float ws_y_max_, float ws_x_min_, float ws_y_min_,
                     float step_, float goal_weight_, float cost_weight_, float lof_distance_, int occ_threshold_, ros::NodeHandlePtr n)
{
    // Pointer to the nodeHandler
    nh = n;
    // Not target initially
    disc_initial = NULL;
    disc_final = NULL;

    // Init asymetric and inflated occupancy matrix
    ws_x_max = round(ws_x_max_ / step_);
    ws_y_max = round(ws_y_max_ / step_);
    ws_x_min = round(ws_x_min_ / step_);
    ws_y_min = round(ws_y_min_ / step_);
    step = step_;
    step_inv = 1.0 / step_;

    Lx = ws_x_max - ws_x_min;
    Ly = ws_y_max - ws_y_min;
    Lx_inv = 1.0 / Lx;
    Ly_inv = 1.0 / Ly;
    matrix_size = Lx * Ly;
    mapParamsConfigured = true;
    printf("ThetaStar (%s): Occupancy Matrix has %d nodes [%d MB]\n", plannerName.c_str(), matrix_size, (int)(matrix_size * sizeof(ThetaStarNode)) / (1024 * 1024));
    discrete_world.resize(matrix_size);

    //Get the value used by the functuion getAverageDistance2Obstacles
    //TODO: Check it work
    if (boost::algorithm::contains("local", plannerName))
    {
        nh->param("/costmap_2d_local/costmap/inflation_layer/cost_scaling_factor", csf, (float)1);
        nh->param("/costmap_2d_local/costmap/robot_radius", rob_rad, (float)0.4);
    }
    else
    {
        nh->param("/costmap_2d/costmap/inflation_layer/cost_scaling_factor", csf, (float)1);
        nh->param("/costmap_2d/costmap/robot_radius", rob_rad, (float)0.4);
    }
    // Optimization
    goal_weight = goal_weight_;
    cost_weight = cost_weight_;
    lof_distance = lof_distance_;
    occ_threshold = occ_threshold_;

    configureMarkers(plannerName, frame_id, step);
    /*
    // Visualitazion Markers
    //char topicPath[100];
    string topicPath = plannerName + "/vis_marker_explored";
    
    //sprintf(topicPath, "%s/vis_marker_explored", plannerName);
    marker_pub_ = nh->advertise<RVizMarker>(topicPath, 1);
    marker.header.frame_id = frame_id; //"world";
    marker.header.stamp = ros::Time();
    marker.ns = "debug";
    marker.id = 66;
    marker.type = RVizMarker::CUBE_LIST;
    marker.action = RVizMarker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0 * step;
    marker.scale.y = 1.0 * step;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    //sprintf(topicPath, "%s/vis_marker_no_lineOfSight", plannerName);
    topicPath = plannerName + "/vis_marker_no_lineOfSight";
    no_los_marker_pub_ = nh->advertise<RVizMarker>(topicPath, 1);
    marker_no_los.header.frame_id = frame_id; //"world";
    marker_no_los.header.stamp = ros::Time();
    marker_no_los.ns = "debug";
    marker_no_los.id = 67;
    marker_no_los.type = RVizMarker::CUBE_LIST;
    marker_no_los.action = RVizMarker::ADD;
    marker_no_los.pose.orientation.w = 1.0;
    marker_no_los.scale.x = 1.0 * step;
    marker_no_los.scale.y = 1.0 * step;
    marker_no_los.color.a = 1.0;
    marker_no_los.color.r = 1.0;
    marker_no_los.color.g = 1.0;
    marker_no_los.color.b = 0.0;
    */
    // if you want a trajectory, its params must be configured using setTrajectoryParams()
    trajectoryParamsConfigured = false;
}
void ThetaStar::configureMarkers(string plannerName, string frame_id, float step)
{
    // Visualitazion Markers
    //char topicPath[100];
    string topicPath = plannerName + "/vis_marker_explored";

    //sprintf(topicPath, "%s/vis_marker_explored", plannerName);
    marker_pub_ = nh->advertise<RVizMarker>(topicPath, 1);
    marker.header.frame_id = frame_id; //"world";
    marker.header.stamp = ros::Time();
    marker.ns = "debug";
    marker.id = 66;
    marker.type = RVizMarker::CUBE_LIST;
    marker.action = RVizMarker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0 * step;
    marker.scale.y = 1.0 * step;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    //sprintf(topicPath, "%s/vis_marker_no_lineOfSight", plannerName);
    topicPath = plannerName + "/vis_marker_no_lineOfSight";
    no_los_marker_pub_ = nh->advertise<RVizMarker>(topicPath, 1);
    marker_no_los.header.frame_id = frame_id; //"world";
    marker_no_los.header.stamp = ros::Time();
    marker_no_los.ns = "debug";
    marker_no_los.id = 67;
    marker_no_los.type = RVizMarker::CUBE_LIST;
    marker_no_los.action = RVizMarker::ADD;
    marker_no_los.pose.orientation.w = 1.0;
    marker_no_los.scale.x = 1.0 * step;
    marker_no_los.scale.y = 1.0 * step;
    marker_no_los.color.a = 1.0;
    marker_no_los.color.r = 1.0;
    marker_no_los.color.g = 1.0;
    marker_no_los.color.b = 0.0;
}
ThetaStar::~ThetaStar()
{
}

void ThetaStar::setTrajectoryParams(float dxy_max_, float dxy_tolerance_, float min_yaw_ahead_)
{
    // Parse params
    dxy_max = dxy_max_;
    dxy_tolerance = dxy_tolerance_;
    min_yaw_ahead = min_yaw_ahead_;
    // Set as configured
    trajectoryParamsConfigured = true;
}

//void ThetaStar::getMap(nav_msgs::OccupancyGrid::ConstPtr &message)
//{
//
//    clearMap();
//    int size = message->info.width * message->info.height;
//    int x, y;
//    trf_x = message->info.origin.position.x;
//    trf_y = message->info.origin.position.y;
//
//    for (unsigned int i = 0; i < size; i++)
//    {
//        getDiscreteWorldPositionFromIndex(x, y, i);
//
//        if (isInside(x, y) && message->data[i] >= occ_threshold)
//            discrete_world[i].notOccupied = false;
//
//        discrete_world[i].cost = message->data[i];
//    }
//}
void ThetaStar::getMap(unsigned char *map)
{
    clearMap();
    int x, y;
    for (unsigned int i = 0; i < matrix_size; i++, map++)
    {
        getDiscreteWorldPositionFromIndex(x, y, i);

        if (isInside(x, y) && *map > 252)
            discrete_world[i].notOccupied = false;

        discrete_world[i].cost = ((float)*map) / 252 * 100;
    }
    // for(auto i = 0; i < discrete_world.size(); i++){
        // ROS_WARN("Cost %d: %f", i, discrete_world[i].cost);
    // }
}
void ThetaStar::getMap(nav_msgs::OccupancyGrid *message)
{
    clearMap();
    int size = message->info.width * message->info.height;
    int x, y;
    trf_x = message->info.origin.position.x;
    trf_y = message->info.origin.position.y;

    for (unsigned int i = 0; i < size; i++)
    {
        getDiscreteWorldPositionFromIndex(x, y, i);

        if (isInside(x, y) && message->data[i] >= occ_threshold)
            discrete_world[i].notOccupied = false;

        discrete_world[i].cost = message->data[i];
    }
}
// Clear the map
void ThetaStar::clearMap()
{
    for (int i = 0; i < matrix_size; i++)
        discrete_world[i].notOccupied = true;
}

bool ThetaStar::setInitialPosition(DiscretePosition p_)
{
    if (isInside(p_.x, p_.y))
    {
        ThetaStartNodeLink *initialNodeInWorld = &discrete_world[getWorldIndex(
            p_.x,
            p_.y)];

        if (initialNodeInWorld->node == NULL)
        {
            initialNodeInWorld->node = new ThetaStarNode();
            initialNodeInWorld->node->point.x = p_.x;
            initialNodeInWorld->node->point.x = p_.y;

            initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
        }
        disc_initial = initialNodeInWorld->node;

        initial_position.x = p_.x * step;
        initial_position.y = p_.y * step;
        disc_initial->point = p_;
        disc_initial->parentNode = disc_initial;

        return true;
    }
    else
    {
        std::cerr << "ThetaStar: Initial point [" << p_.x << ";" << p_.y << "] not valid." << std::endl;
        disc_initial = NULL;
        return false;
    }
}

bool ThetaStar::setInitialPosition(Vector3 p)
{
    initial_position = p;
    DiscretePosition p_ = discretizePosition(p);

    return setInitialPosition(p_);
}

bool ThetaStar::setFinalPosition(DiscretePosition p_)
{
    if (isInside(p_.x, p_.y))
    {
        ThetaStartNodeLink *finalNodeInWorld = &discrete_world[getWorldIndex(
            p_.x,
            p_.y)];

        if (finalNodeInWorld->node == NULL)
        {
            finalNodeInWorld->node = new ThetaStarNode();
            finalNodeInWorld->node->point.x = p_.x;
            finalNodeInWorld->node->point.x = p_.y;

            finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
        }
        disc_final = finalNodeInWorld->node;

        final_position.x = p_.x * step;
        final_position.y = p_.y * step;
        disc_final->point = p_;

        return true;
    }
    else
    {
        ROS_WARN("is not Inside! [%.2f, %.2f] ", p_.x * step, p_.y * step);
        //~ std::cerr << "ThetaStar: Final point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
        disc_final = NULL;
        return false;
    }
}

bool ThetaStar::setFinalPosition(Vector3 p)
{
    DiscretePosition p_ = discretizePosition(p);

    return setFinalPosition(p_);
}

Vector3 ThetaStar::getInitialPosition()
{
    return initial_position;
}

Vector3 ThetaStar::getFinalPosition()
{
    return final_position;
}

bool ThetaStar::lineofsight(ThetaStarNode &p1, ThetaStarNode &p2)
{
    //printf("Checking LofS: [%d, %d, %d] to [%d, %d, %d]\n", p1.point.x, p1.point.y, p1.point.z, p2.point.x, p2.point.y, p2.point.z);

    //min distance for no collision, in discrete space
    float min_distance = 1.0 * step; // 1.9; //sqrt(8);
    int extra_cells = 0;             // min(2.0f,min_distance);

    //distance doble triangle.
    int x0 = max(min(p1.point.x, p2.point.x) - extra_cells, ws_x_min);
    int x1 = min(max(p1.point.x, p2.point.x) + extra_cells, ws_x_max);
    int y0 = max(min(p1.point.y, p2.point.y) - extra_cells, ws_y_min);
    int y1 = min(max(p1.point.y, p2.point.y) + extra_cells, ws_y_max);

#ifdef TESTING_FUNCT
    if (distanceBetween2nodes(p1, p2) > lof_distance / step)
        return false;
#endif

    if (isOccupied(p1) || isOccupied(p2))
        return false;

    float base = distanceBetween2nodes(p1, p2);
    for (int x = x0; x <= x1; x++)
        for (int y = y0; y <= y1; y++)
        {
            //If the point is occupied, we have to calculate distance to the line.
            if (!discrete_world[getWorldIndex(x, y)].notOccupied)
            {
                //If base is zero and node is occcupied, directly does not exist line of sight
                if (base == 0)
                {
#ifdef SEND_NO_LOFS_NODES_MARKERS
                    geometry_msgs::Point p;
                    p.x = p2.point.x * step + trf_x;
                    p.y = p2.point.y * step + trf_y;
                    marker_no_los.header.stamp = ros::Time();
                    marker_no_los.header.seq++;
                    marker_no_los.points.push_back(p);
                    no_los_marker_pub_.publish(marker_no_los);
#endif

                    return false;
                }

                //cout << "Cell is occupied" << endl;
                float a = sqrt(pow(x - p1.point.x, 2) + pow(p1.point.y - y, 2));
                float c = sqrt(pow(p2.point.x - x, 2) + pow(p2.point.y - y, 2));
                float l = (pow(base, 2) + pow(a, 2) - pow(c, 2)) / (2 * base);
                float distance = sqrt(abs(pow(a, 2) - pow(l, 2)));

                if (distance <= min_distance)
                {
#ifdef SEND_NO_LOFS_NODES_MARKERS
                    geometry_msgs::Point p;
                    p.x = p2.point.x * step + trf_x;
                    p.y = p2.point.y * step + trf_y;
                    marker_no_los.header.stamp = ros::Time();
                    marker_no_los.header.seq++;
                    marker_no_los.points.push_back(p);
                    no_los_marker_pub_.publish(marker_no_los);
#endif
                    return false;
                }
            }
        }

    return true;
}

bool ThetaStar::isOccupied(ThetaStarNode n)
{
    return !discrete_world[getWorldIndex(n.point.x, n.point.y)].notOccupied;
}

bool ThetaStar::isInitialPositionOccupied()
{
    if (isOccupied(*disc_initial))
        return true;
    else
        return false;
}

bool ThetaStar::isFinalPositionOccupied()
{
    if (isOccupied(*disc_final))
        return true;
    else
        return false;
}

float ThetaStar::getMapResolution()
{
    return step;
}

DiscretePosition ThetaStar::discretizePosition(Vector3 p)
{
    DiscretePosition res;

    res.x = p.x * step_inv;
    res.y = p.y * step_inv;

    return res;
}

bool ThetaStar::searchInitialPosition2d(float maxDistance)
{
    DiscretePosition init_ = discretizePosition(initial_position); // Start coordinates

    int maxDistance_ = maxDistance / step; // celdas mas lejanas a compromabar a maxDistance meters

    // Check final point, first if is inside the workspace and second if is occupied
    if (setValidInitialPosition(init_))
        return true;

    // Check from z=0 from the near xy ring to the far xy ring
    for (int d = 1; d <= maxDistance_; d++)
    {
        if (searchInitialPositionInXyRing(init_.x, init_.y, d))
            return true;
    }

    return false;
}
bool ThetaStar::searchFinalPosition2d(float maxDistance)
{
    DiscretePosition final_ = discretizePosition(final_position); // Start coordinates

    int maxDistance_ = maxDistance / step; // celdas mas lejanas a compromabar a maxDistance meters

    // Check final point, first if is inside the workspace and second if is occupied
    if (setValidFinalPosition(final_))
        return true;

    // Check from z=0 from the near xy ring to the far xy ring
    for (int d = 1; d <= maxDistance_; d++)
    {
        if (searchFinalPositionInXyRing(final_.x, final_.y, d))
            return true;
    }

    return false;
}
void ThetaStar::setTimeOut(int sec)
{
    timeout = sec;
}

int ThetaStar::getTimeOut()
{
    return timeout;
}

int ThetaStar::computePath(void)
{
    //printf("Calculating...\n");
    if (disc_initial == NULL || disc_final == NULL)
    {
        std::cerr << "ThetaStar: Cannot calculate path. Initial or Final point not valid." << std::endl;
        return 0;
    }

    marker.points.clear();
    marker_no_los.points.clear();

    //Initial point to discrete
    geometry_msgs::Point p;
    p.x = disc_initial->point.x * step;
    p.y = disc_initial->point.y * step;

    marker.points.push_back(p);

    //Initialize data structure. --> clear lists
    ThetaStarNode *erase_node;
    while (!open.empty())
    {
        erase_node = *open.begin();
        open.erase(open.begin());
        if (erase_node != NULL)
        {
            erase_node->nodeInWorld->isInCandidateList = false;
            erase_node->nodeInWorld->isInOpenList = false;
        }
    }
    while (!candidates.empty())
    {
        erase_node = *candidates.begin();
        candidates.erase(candidates.begin());
        if (erase_node != NULL)
        {
            erase_node->nodeInWorld->isInCandidateList = false;
            erase_node->nodeInWorld->isInOpenList = false;
        }
    }
    open.clear();
    candidates.clear();

    disc_initial->distanceFromInitialPoint = 0;
    disc_initial->lineDistanceToFinalPoint = weightedDistanceToGoal(*disc_initial);
    disc_initial->totalDistance = disc_initial->lineDistanceToFinalPoint + 0;
    disc_initial->parentNode = disc_initial;
    //ROS_INFO("Distance to goal: %f", disc_initial->lineDistanceToFinalPoint);
    open.insert(disc_initial);
    disc_initial->nodeInWorld->isInOpenList = true;

    //Initialize loop
    ThetaStarNode *min_distance = disc_initial; // s : current node
    bool noSolution = false;
    long iter = 0;

    if (isOccupied(*disc_final))
    {
        noSolution = true;
        std::cerr << "ThetaStar: Final  point not free." << std::endl;
    }
    if (isOccupied(*disc_initial))
    {
        noSolution = true;
        std::cerr << "ThetaStar: Initial point not free." << std::endl;
        if (searchInitialPositionInXyRing(disc_initial->point.x, disc_initial->point.y, 0.2))
        {
            noSolution = false;
            disc_initial->point.x = getInitialPosition().x;
            disc_initial->point.x = getInitialPosition().y;
        }

        //Fali: Trying to fix the problem of falling inside occupied points
        return -1;
    }

#ifdef PRINT_EXPLORED_NODES_NUMBER
    int expanded_nodes_number = 0;
#endif
    ros::Time last_time_ = ros::Time::now();
    while (!noSolution && (*min_distance) != (*disc_final))
    {
        iter++;
        if (iter % 100 == 0)
        {
            if ((ros::Time::now() - last_time_).toSec() > timeout)
            {
                noSolution = true;
                std::cerr << "Theta Star: Timeout. Iteractions:" << iter << std::endl;
            }
        }

        //If there are more nodes...
        if (!open.empty())
        {
            //Look for minimun distance node
            min_distance = *open.begin();

#ifdef SEND_EXPLORED_NODES_MARKERS
            publishMarker(*min_distance, false);
            //usleep(20000);
#endif

#ifdef PRINT_EXPLORED_NODES_NUMBER
            expanded_nodes_number++;
#endif
            open.erase(open.begin());
            min_distance->nodeInWorld->isInOpenList = false;

            //Insert it in candidate list
            candidates.insert(min_distance);
            min_distance->nodeInWorld->isInCandidateList = true;

            //Look for Neighbors with line of sight
            set<ThetaStarNode *, NodePointerComparator> neighbors;
            getNeighbors(*min_distance, neighbors);

            //Check if exist line of sight.
            SetVertex(*min_distance, neighbors);

            set<ThetaStarNode *, NodePointerComparator>::iterator it_;
            it_ = neighbors.begin();
            ThetaStarNode *new_node;
            while (it_ != neighbors.end())
            {
                new_node = *it_;
                if (!new_node->nodeInWorld->isInCandidateList)
                {
                    if (!new_node->nodeInWorld->isInOpenList)
                    {
                        new_node->totalDistance = std::numeric_limits<float>::max();
                        new_node->lineDistanceToFinalPoint = weightedDistanceToGoal(*new_node);
                        new_node->parentNode = min_distance;
                    }
                    UpdateVertex(*min_distance, *new_node);
                }
                it_++;
            }
        }
        else //If there are not nodes, do not exist solution
        {
            noSolution = true;
        }
    }

#ifdef SEND_EXPLORED_NODES_MARKERS
    publishMarker(*min_distance, true);
#endif

#ifdef PRINT_EXPLORED_NODES_NUMBER
    ROS_INFO("Theta Star: Expanded nodes: %d", expanded_nodes_number);
#endif

    //Path finished, get final path
    last_path.clear();
    ThetaStarNode *path_point;
    Vector3 point;

    path_point = min_distance;
    if (noSolution)
    {
        std::cerr << "Imposible to calculate a solution" << std::endl;
    }

    while (!noSolution && path_point != disc_initial)
    {
        point.x = path_point->point.x * step;
        point.y = path_point->point.y * step;

        last_path.insert(last_path.begin(), point);

        cost_path += path_point->nodeInWorld->cost;

        path_point = path_point->parentNode;
        if (!path_point || (path_point == path_point->parentNode && path_point != disc_initial))
        {
            last_path.clear();
            break;
        }
    }
    cost_path /= last_path.size();
    computeAverageDist2Obs();
    //ROS_WARN("Expanded Nodes: %d",expanded_nodes_number);
    return last_path.size();
}

vector<Vector3> ThetaStar::getCurrentPath()
{
    return last_path;
}
int ThetaStar::getCurrentPathNPoints()
{
    return last_path.size();
}
//!Test function to deduce the distance from the cost
//TODO: Check it work
void ThetaStar::computeAverageDist2Obs()
{
    average_dist_to_obst = 1 / csf * log(occ_threshold / cost_path) + rob_rad;
}
float ThetaStar::getAvDist2Obs()
{
    return average_dist_to_obst;
}
bool ThetaStar::getTrajectoryYawFixed(Trajectory &trajectory, double fixed_yaw)
{
    if (!trajectoryParamsConfigured)
        return false;

    // number of PATH points without the initial point
    int n_path = last_path.size();

    // Time respect first path/trajectory position (0.0)

    // Trajectory point to fill the complete trajectory msg
    TrajectoryPoint trajectory_point;
    trajectory_point.transforms.resize(1);
    trajectory_point.velocities.resize(1);
    trajectory_point.accelerations.resize(1);

    // loop last and next path positions
    Vector3 last_position = initial_position;
    Vector3 next_position = last_path[0];
    Vector3 middle_position, dir;

    double mod = getHorizontalNorm(last_path[0].x - initial_position.x, last_path[0].y - initial_position.y);
    dir.x = (last_path[0].x - initial_position.x) / mod;
    dir.y = (last_path[0].y - initial_position.y) / mod;

    // entire path loop
    for (int i = 1; i <= n_path; i++)
    {

        // two path points loop
        while (getHorizontalNorm(fabs(last_position.x - next_position.x), fabs(last_position.y - next_position.y)) > dxy_tolerance)
        {

            middle_position.x = last_position.x + dxy_tolerance * dir.x;
            middle_position.y = last_position.y + dxy_tolerance * dir.y;

            // Set path position (last_position is the original last_position or the last middle_position if it exists)
            setPositionYawAndTime(trajectory_point, middle_position, fixed_yaw);
            trajectory.points.push_back(trajectory_point);

            //.. to the algorihtm
            last_position = middle_position;
        }
        if (i == n_path)
        {
            break;
        }

        setPositionYawAndTime(trajectory_point, next_position, fixed_yaw);
        trajectory.points.push_back(trajectory_point);

        last_position = next_position;
        next_position = last_path[i];
        mod = getHorizontalNorm(fabs(last_position.x - next_position.x), fabs(last_position.y - next_position.y));
        dir.x = (next_position.x - last_position.x) / mod;
        dir.y = (next_position.y - last_position.y) / mod;
    }

    return true;
}

bool ThetaStar::getTrajectoryYawAtTime(Trajectory &trajectory, Transform init_pose)
{
    if (!trajectoryParamsConfigured)
        return false;

    // Get the current yaw (odom)
    double yaw_odom = getYawFromQuat(init_pose.rotation);

    // Get Trajectory [X,Y,Z,T] without yaw changes
    getTrajectoryYawFixed(trajectory, yaw_odom);

    // Transform the previous trajectory setting the Yaw ahead at Time and the neccesay time increment for get it
    double yaw = yaw_odom;
    double dyaw = 0.0;
    double last_yaw = yaw;
    tf::Quaternion q;

    for (int k = 0; k < trajectory.points.size(); k++)
    {
        // get yaw ahead for the next segment, only if the translation is long enought
        if (k == 0)
        {
            if (getHorizontalNorm(trajectory.points[k].transforms[0].translation.x - initial_position.x, trajectory.points[k].transforms[0].translation.y - initial_position.y) >= min_yaw_ahead)
                yaw = atan2(trajectory.points[k].transforms[0].translation.y - initial_position.y, trajectory.points[k].transforms[0].translation.x - initial_position.x);
        }
        else
        {
            if (getHorizontalDistance(trajectory.points[k], trajectory.points[k - 1]) >= min_yaw_ahead)
                yaw = atan2(trajectory.points[k].transforms[0].translation.y - trajectory.points[k - 1].transforms[0].translation.y, trajectory.points[k].transforms[0].translation.x - trajectory.points[k - 1].transforms[0].translation.x);
        }

        dyaw = getDyaw(yaw, last_yaw);

        last_yaw = yaw;
        q.setRPY(0.0, 0.0, yaw);
        trajectory.points[k].transforms[0].rotation.x = q.x();
        trajectory.points[k].transforms[0].rotation.y = q.y();
        trajectory.points[k].transforms[0].rotation.z = q.z();
        trajectory.points[k].transforms[0].rotation.w = q.w();
    }

    return true;
}

bool ThetaStar::getTrajectoryYawInAdvance(Trajectory &trajectory, Transform init_pose)
{
    if (!trajectoryParamsConfigured)
        return false;

    // Trajectory point to fill the complete trajectory msg
    TrajectoryPoint trajectory_point;
    trajectory_point.transforms.resize(1);
    trajectory_point.velocities.resize(1);
    trajectory_point.accelerations.resize(1);

    // Get the current yaw (odom)
    double yaw_odom = getYawFromQuat(init_pose.rotation);

    // Get Trajectory [X,Y,Z,T] without yaw changes
    getTrajectoryYawFixed(trajectory, yaw_odom);

    //print actual trajectory vector state
    //printfTrajectory(trajectory, "Trajectory_state_0");

    // Set First wp as initial position + yaw ahead = dir(odom,wp1) IF the horizontal movement is enough.
    // If not try with next wp. If not let yaw reference as yaw_odom and not se this first wp
    int traj_size = trajectory.points.size();
    double yaw = yaw_odom;
    double dyaw = 0.0;
    double last_yaw = yaw;
    int i = 0;
    //printf(PRINTF_GREEN"\nTrajectory size = %d\n", traj_size);
    while (getHorizontalNorm(trajectory.points[i].transforms[0].translation.x - initial_position.x, trajectory.points[i].transforms[0].translation.y - initial_position.y) < min_yaw_ahead)
    {
        //printf(PRINTF_GREEN"Intial Pos: [%.4f, %.4f], Tajectory Point: [%.4f,%.4f]\n", initial_position.x, initial_position.y, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y);
        i++;

        if (i > traj_size - 1)
        {
            //printf(PRINTF_GREEN"break\n");
            i = -1;
            break;
        }
    }
    if (i >= 0)
    {
        yaw = atan2(trajectory.points[i].transforms[0].translation.y - initial_position.y, trajectory.points[i].transforms[0].translation.x - initial_position.x);
        //printf("i = %d, Setting yaw = %.4f from [%.4f, %.4f] to [%.4f, %.4f]\n", i, yaw, initial_position.x, initial_position.y, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y);
        dyaw = getDyaw(yaw, yaw_odom);
        last_yaw = yaw;
        setPositionYawAndTime(trajectory_point, initial_position, yaw);
        trajectory.points.insert(trajectory.points.begin(), trajectory_point);
    }

    //print actual trajectory vector state
    //printfTrajectory(trajectory, "Trajectory_state_1");

    // Transform the previous trajectory setting the Yaw ahead in Advance and the neccesay time increment for get it
    tf::Quaternion q;      // yaw as quaternion to the msg
    double time_inc = 0.0; // time increment if yaw reference changes
    for (int k = 1; k < trajectory.points.size() - 1; k++)
    {
        // Search the next minimum horizontal movement to set as a yaw reference
        i = 0; // index to search the next bigger enough segment to compute the yaw reference (eliminating yaw references for small horizontal increments)
        while (getHorizontalDistance(trajectory.points[k + i + 1], trajectory.points[k + i]) < min_yaw_ahead)
        {
            i++;

            if (k + i + 1 >= trajectory.points.size())
            {
                // if all segments are shorter than min_yaw_ahead, doesn't change the yaw reference because it would be a only-vertical movement
                i = -1;
                break;
            }
        }

        // get yaw ahead for the next segment, yaw increment and neccesary time for this turn
        if (i >= 0)
            yaw = atan2(trajectory.points[k + 1 + i].transforms[0].translation.y - trajectory.points[k + i].transforms[0].translation.y, trajectory.points[k + 1 + i].transforms[0].translation.x - trajectory.points[k + i].transforms[0].translation.x);

        dyaw = getDyaw(yaw, last_yaw);
        last_yaw = yaw;
        // set as quaternion
        q.setRPY(0.0, 0.0, yaw);
        trajectory.points[k].transforms[0].rotation.x = q.x();
        trajectory.points[k].transforms[0].rotation.y = q.y();
        trajectory.points[k].transforms[0].rotation.z = q.z();
        trajectory.points[k].transforms[0].rotation.w = q.w();
        // calculate time increment
        // check if neccesary yaw turn time is bigger than for the position increment
        //Este trozo me estaba dando problemas, vamos a omitirlo de momento porque el time increment
        /*if( time_inc > 0.0 )
		{
			ROS_INFO("[%d] Exist time increment: %f",  k, time_inc);
			// update time for this waypoint and all next waypoints
			for( int j = k; j<trajectory.points.size(); j++ )
			{
				trajectory.points[j].time_from_start += ros::Duration(time_inc);
			}
			printfTrajectory(trajectory, "Trajectory_state_middle");
		}*/
    }
    // last waypoint yaw reference (same that second last, so yaw does not change, so time does not change)
    q.setRPY(0.0, 0.0, yaw); // Note: If it's only one wp the 'for' statement does not execute nothing, so its neccesary this line
    trajectory.points.back().transforms[0].rotation.x = q.x();
    trajectory.points.back().transforms[0].rotation.y = q.y();
    trajectory.points.back().transforms[0].rotation.z = q.z();
    trajectory.points.back().transforms[0].rotation.w = q.w();

    return true;
}

bool ThetaStar::getTrajectoryYawInAdvanceWithFinalYaw(Trajectory &trajectory, Transform init_pose, double final_yaw_ref)
{
    if (!trajectoryParamsConfigured)
        return false;

    // Get the current trajectory with yaw in advance
    getTrajectoryYawInAdvance(trajectory, init_pose);

    // Modify the last yaw reference and the time if it's neccesary
    double last_wp_yaw = getYawFromQuat(trajectory.points.back().transforms[0].rotation);
    double dyaw = fabs(final_yaw_ref - last_wp_yaw);
    if (dyaw > 0.0)
    {
        // Set new yaw
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, final_yaw_ref);
        trajectory.points.back().transforms[0].rotation.x = q.x();
        trajectory.points.back().transforms[0].rotation.y = q.y();
        trajectory.points.back().transforms[0].rotation.z = q.z();
        trajectory.points.back().transforms[0].rotation.w = q.w();

        // Check if its neccesary more time (only if trajectory is longer than only one wp)
        int traj_size = trajectory.points.size();
        if (traj_size > 1)
        {
            double pre_last_yaw = getYawFromQuat(trajectory.points.at(traj_size - 2).transforms[0].rotation);
            dyaw = getDyaw(final_yaw_ref, pre_last_yaw);
        }
    }
}

void ThetaStar::getNeighbors(ThetaStarNode &node, set<ThetaStarNode *, NodePointerComparator> &neighbors)
{
    neighbors.clear();

    DiscretePosition node_temp;

    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            /*
                 *Ignore ourselves
                */
            if (i != 0 || j != 0)
            {
                node_temp.x = node.point.x + i;
                node_temp.y = node.point.y + j;

                if (isInside(node_temp.x, node_temp.y))
                {
                    int nodeInWorld = getWorldIndex(node_temp.x, node_temp.y);

                    ThetaStartNodeLink *new_neighbor = &discrete_world[nodeInWorld];
                    if (new_neighbor->node == NULL)
                    {
                        new_neighbor->node = new ThetaStarNode();
                        new_neighbor->node->point.x = node_temp.x;
                        new_neighbor->node->point.y = node_temp.y;
                        new_neighbor->node->nodeInWorld = new_neighbor;
                        new_neighbor->node->parentNode = &node;
                    }
                    if (new_neighbor->isInCandidateList || lineofsight(node, *new_neighbor->node))
                    {
                        neighbors.insert(new_neighbor->node);
                    }
                }
            }
        }
    }
}

void ThetaStar::publishMarker(ThetaStarNode &s, bool publish)
{
    geometry_msgs::Point point;
    point.x = s.point.x * step + trf_x;
    point.y = s.point.y * step + trf_y;

    marker.points.push_back(point);

#ifdef STEP_BY_STEP
    publish = true;
#endif

    if (publish)
    {
        marker.header.stamp = ros::Time();
        marker.header.seq++;
        marker_pub_.publish(marker);
    }

#ifdef STEP_BY_STEP
    usleep(1e3);
#endif
}

float ThetaStar::distanceToGoal(ThetaStarNode node)
{
    return sqrt(pow(disc_final->point.x - node.point.x, 2) +
                pow(disc_final->point.y - node.point.y, 2));
}

float ThetaStar::weightedDistanceToGoal(ThetaStarNode node)
{
    return goal_weight * sqrt(pow(disc_final->point.x - node.point.x, 2) +
                              pow(disc_final->point.y - node.point.y, 2));
}

float ThetaStar::distanceBetween2nodes(ThetaStarNode &n1, ThetaStarNode &n2)
{
    return sqrt(pow(n1.point.x - n2.point.x, 2) +
                pow(n1.point.y - n2.point.y, 2));
}

float ThetaStar::weightedDistanceBetween2nodes(ThetaStarNode &n1, ThetaStarNode &n2)
{
    //Aï¿½adido un factor multiplicativo  discrete_world[i].cost*
    int i = getWorldIndex(n2.point.x, n2.point.y);

    return discrete_world[i].cost * cost_weight + sqrt(pow(n1.point.x - n2.point.x, 2) +
                                                       pow(n1.point.y - n2.point.y, 2));
}

float ThetaStar::distanceFromInitialPoint(ThetaStarNode node, ThetaStarNode parent)
{
    float res;
    if (isOccupied(node))
        res = std::numeric_limits<float>::max();
    else if (parent.distanceFromInitialPoint == std::numeric_limits<float>::max())
        res = parent.distanceFromInitialPoint;
    else
    {
        res = parent.distanceFromInitialPoint + (sqrt(pow(node.point.x - parent.point.x, 2) +
                                                      pow(node.point.y - parent.point.y, 2)));
    }

    return res;
}

float ThetaStar::weightedDistanceFromInitialPoint(ThetaStarNode node, ThetaStarNode parent)
{
    float res;
    int i = getWorldIndex(node.point.x, node.point.y);
    if (isOccupied(node))
        res = std::numeric_limits<float>::max();
    else if (parent.distanceFromInitialPoint == std::numeric_limits<float>::max())
        res = parent.distanceFromInitialPoint;
    else
    { //Aï¿½adido el coste del costmap para el termino de distancia entre padre e hijodiscrete_world[i].cost*
        res = parent.distanceFromInitialPoint + discrete_world[i].cost * cost_weight + (sqrt(pow(node.point.x - parent.point.x, 2) + pow(node.point.y - parent.point.y, 2)));
    }

    return res;
}

void ThetaStar::ComputeCost(ThetaStarNode &s, ThetaStarNode &s2)
{
    double distanceParent2 = weightedDistanceBetween2nodes((*s.parentNode), s2);
    if (s.parentNode->distanceFromInitialPoint + distanceParent2 + s2.lineDistanceToFinalPoint < s2.totalDistance)
    {
        s2.parentNode = s.parentNode;
        s2.distanceFromInitialPoint = weightedDistanceFromInitialPoint(s2, *s2.parentNode);
        s2.totalDistance = s2.distanceFromInitialPoint + s2.lineDistanceToFinalPoint;
    }
}

void ThetaStar::UpdateVertex(ThetaStarNode &s, ThetaStarNode &s2)
{
    float g_old = s2.totalDistance;

    ComputeCost(s, s2);
    if (s2.totalDistance < g_old)
    {
        if (s2.nodeInWorld->isInOpenList)
        {
            open.erase(&s2);
        }
        open.insert(&s2);
    }
}

void ThetaStar::SetVertex(ThetaStarNode &s, set<ThetaStarNode *, NodePointerComparator> &neighbors)
{
    if (!lineofsight(*s.parentNode, s))
    {
        float g_value = std::numeric_limits<float>::max();
        ThetaStarNode *parentCandidate = NULL;
        set<ThetaStarNode *, NodePointerComparator>::iterator it_;
        it_ = neighbors.begin();
        ThetaStarNode *new_node;
        bool candidatesNotEmpty = false;
        bool hasLessNumber = false;

        while (it_ != neighbors.end())
        {
            new_node = *it_;
            if (new_node->nodeInWorld->isInCandidateList)
            {
                candidatesNotEmpty = true;
                float g_new = new_node->distanceFromInitialPoint + weightedDistanceBetween2nodes(*new_node, s);
                if (g_new < g_value)
                {
                    hasLessNumber = true;
                    g_value = g_new;
                    parentCandidate = new_node;
                }
            }
            it_++;
        }

#ifdef PRINT_SETVERTEX_FAILS
        if (parentCandidate == NULL)
        {
            std::cerr << "Parent not found" << std::endl;
            if (candidatesNotEmpty)
            {
                std::cerr << "There are one candidate" << std::endl;
            }

            if (hasLessNumber)
            {
                std::cerr << "Is less than..." << std::endl;
            }
        }
#endif
        if (parentCandidate != NULL)
        {
            s.parentNode = parentCandidate;
        }
        s.distanceFromInitialPoint = g_value;
        s.totalDistance = g_value + s.lineDistanceToFinalPoint;
    }
}

double ThetaStar::g(ThetaStarNode &s)
{
    if (s.parentNode == NULL)
    {
        std::cerr << "Error: Parent is null" << std::endl;
        return std::numeric_limits<double>::max();
    }
    else
    {
        float distanceFromInitial_ = weightedDistanceFromInitialPoint(s, *s.parentNode);
        float distanceToGoal_ = weightedDistanceToGoal(s);
        return distanceFromInitial_ + distanceToGoal_;
    }
}

void ThetaStar::setPositionYawAndTime(TrajectoryPoint &trajectory_point, Vector3 position, double _yaw)
{
    tfScalar roll = 0.0;
    tfScalar pitch = 0.0;
    tfScalar yaw = _yaw;
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    trajectory_point.transforms[0].translation.x = position.x;
    trajectory_point.transforms[0].translation.y = position.y;
    trajectory_point.transforms[0].translation.z = position.z;
    trajectory_point.transforms[0].rotation.x = q.x();
    trajectory_point.transforms[0].rotation.y = q.y();
    trajectory_point.transforms[0].rotation.z = q.z();
    trajectory_point.transforms[0].rotation.w = q.w();
    trajectory_point.velocities[0].linear.x = 0.0;
    trajectory_point.velocities[0].linear.y = 0.0;
    trajectory_point.velocities[0].linear.z = 0.0;
    trajectory_point.accelerations[0].linear.x = 0.0;
    trajectory_point.accelerations[0].linear.y = 0.0;
    trajectory_point.accelerations[0].linear.z = 0.0;
}
double ThetaStar::getHorizontalNorm(double x, double y)
{
    return sqrtf(x * x + y * y);
}

double ThetaStar::getHorizontalDistance(TrajectoryPoint P1, TrajectoryPoint P2)
{
    return getHorizontalNorm(P1.transforms[0].translation.x - P2.transforms[0].translation.x, P1.transforms[0].translation.y - P2.transforms[0].translation.y);
}

float ThetaStar::getYawFromQuat(Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y;
}

double ThetaStar::getDyaw(double next_yaw, double last_yaw)
{
    double dyaw = fabs(next_yaw - last_yaw);
    if (dyaw > M_PI)
        dyaw = fabs(dyaw - 2.0 * M_PI);

    return dyaw;
}

void ThetaStar::printfTrajectory(Trajectory trajectory, string trajectory_name)
{
    printf(PRINTF_YELLOW "%s trajectory [%d]:\n", trajectory_name.c_str(), (int)trajectory.points.size());

    for (unsigned int i = 0; i < trajectory.points.size(); i++)
    {
        double yaw = getYawFromQuat(trajectory.points[i].transforms[0].rotation);
        printf(PRINTF_BLUE "\t %d: [%.3f, %.3f] m\t[%f] deg\t [%.2f] sec\n", i, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y, yaw / 3.141592 * 180, trajectory.points[i].time_from_start.toSec());
    }

    printf(PRINTF_REGULAR);
}

void ThetaStar::confPrintRosWarn(bool print)
{
    PRINT_WARNINGS = print;
}

} // namespace PathPlanners
