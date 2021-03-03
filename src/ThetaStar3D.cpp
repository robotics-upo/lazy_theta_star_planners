/*
 * Copyright 2016 Ricardo Ragel de la Torre, GRVC, Univ. of Seville, Spain
 *
 * Resume: 	ThetaStar Class definitions.
 * 			Motion Path Planner based on the Theta Star algoritm: 
 * 			"Weighted Lazy Theta Star with Optimization" and different
 * 			methods to get a colission-free path [x,y,z] or 
 * 			trajectory [xyz, yaw, time].
 */

#include <theta_star/ThetaStar3D.hpp>

namespace PathPlanners
{

// Uncomment to printf the explored nodes number
#define PRINT_EXPLORED_NODES_NUMBER
// Uncomment to get the explored nodes (at time or slowly step_by_step)
#define SEND_EXPLORED_NODES_MARKERS
// #define STEP_BY_STEP
// Uncomment to get non-LineOfSight visual markers
//#define SEND_NO_LOFS_NODES_MARKERS
// Uncomment to printf octree leaf free and occupied
//#define PRINT_OCTREE_STATS
// Uncomment to printf if setVertex() fails at no-LofS
//#define PRINT_SETVERTEX_FAILS
#define TESTING_FUNC
//*****************************************************************
// 				ThetaStar Algorithm Class Definitions
//*****************************************************************
// Default constructor
ThetaStar3D::ThetaStar3D()
{
}

// Constructor with arguments
ThetaStar3D::ThetaStar3D(std::string plannerName, std::string frame_id, float ws_x_max_, float ws_y_max_,
float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_, float step_, float h_inflation_, float v_inflation_, float goal_weight_,
float z_weight_cost_, float z_not_inflate_, ros::NodeHandlePtr n){
	// Call to initialization
	init(plannerName, frame_id, ws_x_max_, ws_y_max_, ws_z_max_, ws_x_min_, ws_y_min_, ws_z_min_, step_, h_inflation_, v_inflation_, goal_weight_, z_weight_cost_, z_not_inflate_, n);
}

// Initialization: creates the occupancy matrix (discrete nodes) from the bounding box sizes, resolution, inflation and optimization arguments
void ThetaStar3D::init(std::string plannerName, std::string frame_id, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_,
					   float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, ros::NodeHandlePtr n)
{
	// Pointer to the nodeHandler
	nh = n;

	// Not target initially
	disc_initial = NULL;
	disc_final = NULL;
	minR=0;
	// by default ~not timeout
	timeout = 100;
	// Init asymetric and inflated occupancy matrix
	ws_x_max = ((ws_x_max_ / step_) + 1);
	ws_y_max = ((ws_y_max_ / step_) + 1);
	ws_z_max = ((ws_z_max_ / step_) + 1);
	ws_x_min = ((ws_x_min_ / step_) - 1);
	ws_y_min = ((ws_y_min_ / step_) - 1);
	ws_z_min = ((ws_z_min_ / step_) - 1);

	step = step_;
	step_inv = 1.0 / step_;
	h_inflation = (int)(h_inflation_ / step_);
	v_inflation = (int)(v_inflation_ / step_);
	ws_x_max_inflated = (ws_x_max + 2 * h_inflation);
	ws_y_max_inflated = (ws_y_max + 2 * h_inflation);
	ws_z_max_inflated = (ws_z_max + 2 * v_inflation);
	ws_x_min_inflated = (ws_x_min - 2 * h_inflation);
	ws_y_min_inflated = (ws_y_min - 2 * h_inflation);
	ws_z_min_inflated = (ws_z_min - 2 * v_inflation);
	matrix_size = (abs(ws_x_max_inflated) - ws_x_min_inflated + 1) * (abs(ws_y_max_inflated) - ws_y_min_inflated + 1) * (abs(ws_z_max_inflated) - ws_z_min_inflated + 1);
	discrete_world.resize(matrix_size);
	printf("ThetaStar (%s): Occupancy Matrix has %d nodes [%lu MB]\n", plannerName.c_str(), matrix_size, (uint_fast32_t)(matrix_size * sizeof(ThetaStarNode3D)) / (1024 * 1024));
	Lx = ws_x_max_inflated - ws_x_min_inflated + 1;
	Ly = ws_y_max_inflated - ws_y_min_inflated + 1;
	Lz = ws_z_max_inflated - ws_z_min_inflated + 1;
	Lx_inv = 1.0 / Lx;
	Ly_inv = 1.0 / Ly;
	Lz_inv = 1.0 / Lz;
	// Optimization
	goal_weight = goal_weight_;
	z_weight_cost = z_weight_cost_;
	// Floor condition
	z_not_inflate = z_not_inflate_;
	// Visualitazion Markers
	std::string topicPath;
	topicPath = plannerName + "/vis_marker_explored";

	marker_pub_ = nh->advertise<RVizMarker>(topicPath.c_str(), 1);
	marker.header.frame_id = frame_id; //"world";
	marker.header.stamp = ros::Time();
	marker.ns = "debug";
	marker.id = 66;
	marker.type = RVizMarker::CUBE_LIST;
	marker.action = RVizMarker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1.0 * step;
	marker.scale.y = 1.0 * step;
	marker.scale.z = 1.0 * step;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	topicPath = plannerName + "/vis_marker_occupancy";

	occupancy_marker_pub_ = nh->advertise<PointCloud>(topicPath.c_str(), 1, true);
	occupancy_marker.header.frame_id = frame_id; // "world";

	topicPath = plannerName + "/vis_marker_no_lineOfSight";

	no_los_marker_pub_ = nh->advertise<RVizMarker>(topicPath.c_str(), 1);
	marker_no_los.header.frame_id = frame_id; //"world";
	marker_no_los.header.stamp = ros::Time();
	marker_no_los.ns = "debug";
	marker_no_los.id = 66;
	marker_no_los.type = RVizMarker::CUBE_LIST;
	marker_no_los.action = RVizMarker::ADD;
	marker_no_los.pose.orientation.w = 1.0;
	marker_no_los.scale.x = 1.0 * step;
	marker_no_los.scale.y = 1.0 * step;
	marker_no_los.scale.z = 1.0 * step;
	marker_no_los.color.a = 1.0;
	marker_no_los.color.r = 1.0;
	marker_no_los.color.g = 1.0;
	marker_no_los.color.b = 0.0;
	// if you want a trajectory, its params must be configured using setTrajectoryParams()
	trajectoryParamsConfigured = false;
    m_grid3d.reset(new Grid3d);

	closest_distances_pub_  = nh->advertise<std_msgs::Float32MultiArray>("/theta_star_3d/closest_distances", 1);
	closest_distance_pub_   = nh->advertise<std_msgs::Float32>("/theta_star_3d/closest_distance", 1);
    set_cost_params_server_ = nh->advertiseService("/theta_star_3d/set_costs_values", &ThetaStar3D::setCostsParamsSrv, this);
    switch_astar_server_ = nh->advertiseService("/theta_star_3d/switch_astar", &ThetaStar3D::switchAstar, this);

	nh->param("/theta_star_3d/astar_mode", use_astar, false);
	if(use_astar){
		goal_weight = 1.0; //No goal weight
		z_weight_cost  = 1.0;
		cost_weight = 0.0;
		std::cout<<"Using A* Algorithm" << std::endl;
	}
	nh->param("/theta_star_3d/save_data_to_file", save_data_, false);
	nh->param("/theta_star_3d/data_filename", data_file, (std::string)"");
	if(save_data_){
		out_file_data_.open(data_file, std::ofstream::app);
	}
	
}

ThetaStar3D::~ThetaStar3D()
{
	if(save_data_){
		out_file_data_.close();
	}
}
void ThetaStar3D::setTrajectoryParams(float dxy_max_, float dz_max_, float dxyz_tolerance_, float vm_xy_, float vm_z_, float vm_xy_1_, float vm_z_1_, float w_yaw_, float min_yaw_ahead_)
{
	// Parse params
	dxy_max = dxy_max_;
	dz_max = dz_max_;
	dxyz_tolerance = dxyz_tolerance_;
	vm_xy = vm_xy_;
	vm_z = vm_z_;
	vm_xy_1 = vm_xy_1_;
	vm_z_1 = vm_z_1_;
	w_yaw = w_yaw_;
	min_yaw_ahead = min_yaw_ahead_;

	// Set as configured
	trajectoryParamsConfigured = true;
}

void ThetaStar3D::updateMap(octomap_msgs::OctomapConstPtr message)
{
	// Clear current map in the discrete occupancy
	clearMap();

	// Read occupation data from the octomap_server
	//octomap_msgs::binaryMsgToMap(message));
	map = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*message);
	/*
     * Update discrete world with the read octomap data
     */

	// Occupied and not occupied points, only for debug
	u_int64_t occupied_leafs = 0, free_leafs = 0;
	int nit = 0;

	// Read from first to the last leaf of the tree set its xyz (discretized) if it is occupied into the occupancy matrix

	if (map->begin_leafs() != NULL)
	{
		for (octomap::OcTree::leaf_iterator it = map->begin_leafs(), end = map->end_leafs(); it != end; ++it)
		{
			if (map->isNodeOccupied(*it))
			{
				// Get occupied cells
				float x_w = it.getX();
				float y_w = it.getY();
				float z_w = it.getZ();

				// Exact discretization
				int x_ = (int)(x_w * step_inv);
				int y_ = (int)(y_w * step_inv);
				int z_ = (int)(z_w * step_inv);

				// Set as occupied in the discrete matrix
				//ROS_INFO("[%.2f, %.2f, %.2f] step_inv: %.2f", x_w, y_w, z_w, step_inv);
				//ROS_INFO("Disc [%d, %d, %d]", x_, y_, z_);
				//usleep(5e4);
				if (isInside(x_, y_, z_))
				{
					++nit;
					//ROS_INFO(PRINTF_RED"[%.2f, %.2f, %.2f] step_inv: %.2f", x_w, y_w, z_w, step_inv);
					//ROS_INFO(PRINTF_RED"Disc [%d, %d, %d]", x_, y_, z_);
					//usleep(5e4);
					unsigned int world_index_ = getWorldIndex(x_, y_, z_);
					discrete_world[world_index_].notOccupied = false;
					//ROS_INFO(PRINTF_RED"Inside: [%.2f, %.2f, %.2f]", x_w,y_w, z_w);
					//
					// Inflates nodes
					if (h_inflation >= step || v_inflation >= step)
					{
						if (z_w > z_not_inflate)
							//inflateNodeAsCube(x_, y_, z_);
							//inflateNodeAsCylinder(x_, y_, z_);
							inflateNodeAsXyRectangle(x_, y_, z_);

						else
							inflateNodeAsCylinder(x_, y_, z_);
					}
				}

#ifdef PRINT_OCTREE_STATS
				occupied_leafs++;
#endif
			}
			else
			{
#ifdef PRINT_OCTREE_STATS
				free_leafs++;
#endif
			}
		}
	}

#ifdef PRINT_OCTREE_STATS
	std::cout << "Occupied cells: " << occupied_leafs << " NO occupied cells: " << free_leafs << "It counter: " << nit << std::endl;
	ROS_INFO("Occupied cells: %d\t No occupied cells: %d", occupied_leafs, free_leafs);
#endif
}
void ThetaStar3D::updateMap(PointCloud cloud)
{
	/*
     * Update discrete world with the Point Cloud = ocuppieds cells
     */
	clearMap();
	double dist=0;
	
	BOOST_FOREACH (const pcl::PointXYZ &p, cloud.points)
	{
		// Get occupied points
		//const pcl::PointXYZ &p = cloud.points[it];
		float x_w = p.x;
		float y_w = p.y;
		float z_w = p.z;

		// Exact discretization
		int x_ = (int)(x_w * step_inv);
		int y_ = (int)(y_w * step_inv);
		int z_ = (int)(z_w * step_inv);

		if (isInside(x_, y_, z_))
		{
			unsigned int world_index_ = getWorldIndex(x_, y_, z_);
			//dist=sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
			//if(dist < minR){
			//	discrete_world[world_index_].notOccupied = true;
			//	continue;
			//}
			//else
			discrete_world[world_index_].notOccupied = false;
			
			// Inflates nodes
			if (h_inflation * step >= step || v_inflation * step >= step)
			{
				if (z_w > z_not_inflate)
				{
					inflateNodeAsCube(x_, y_, z_);
					//inflateNodeAsCylinder(x_, y_, z_);
					// inflateNodeAsXyRectangle(x_, y_, z_);
				}
				else
				{
					//inflateNodeAsXyRectangle(x_, y_, z_);
					inflateNodeAsCube(x_, y_, z_);
				}
			}
		}
	}
	int occ = 0;
	int total = discrete_world.size();
	for(auto &it: discrete_world){
		if(!it.notOccupied) ++occ;
	}
	ROS_INFO("Discrete world size: %d, Occupied: %d", total, occ);
}

void ThetaStar3D::updateMap(const PointCloud::ConstPtr &map)
{
	/*
     * Update discrete world with the Point Cloud = ocuppieds cells
     */
	BOOST_FOREACH (const pcl::PointXYZ &p, map->points)
	{
		// Get occupied points
		//const pcl::PointXYZ &p = cloud->points[it];
		float x_w = p.x;
		float y_w = p.y;
		float z_w = p.z;

		// Exact discretization
		int x_ = (int)(x_w * step_inv);
		int y_ = (int)(y_w * step_inv);
		int z_ = (int)(z_w * step_inv);

		if (isInside(x_, y_, z_))
		{
			unsigned int world_index_ = getWorldIndex(x_, y_, z_);
			discrete_world[world_index_].notOccupied = false;

			// Inflates nodes
			if (h_inflation * step >= step || v_inflation * step >= step)
			{
				if (z_w > z_not_inflate)
				{
					inflateNodeAsCube(x_, y_, z_);
					//inflateNodeAsCylinder(x_, y_, z_);
					// inflateNodeAsXyRectangle(x_, y_, z_);
				}
				else
				{
					//inflateNodeAsXyRectangle(x_, y_, z_);
					inflateNodeAsCube(x_, y_, z_);
				}
			}
		}
	}
}

// Clear the map
void ThetaStar3D::clearMap()
{
	for (int i = 0; i < matrix_size; i++)
	{
		discrete_world[i].notOccupied = true;
	}
}

// Puplish to RViz the occupancy map matrix as point cloud markers
void ThetaStar3D::publishOccupationMarkersMap()
{
	//~ occupancy_marker.points.clear();
	occupancy_marker.clear();
	for (int i = ws_x_min_inflated; i <= ws_x_max_inflated; i++)
		for (int j = ws_y_min_inflated; j <= ws_y_max_inflated; j++)
			for (int k = ws_z_min_inflated; k <= ws_z_max_inflated; k++)
			{
				unsigned int matrixIndex = getWorldIndex(i, j, k);

				if (!discrete_world[matrixIndex].notOccupied)
				{
					//~ geometry_msgs::Point point;
					pcl::PointXYZ point;
					point.x = i * step;
					point.y = j * step;
					point.z = k * step;
					//~ occupancy_marker.points.push_back(point);
					occupancy_marker.push_back(point);
				}
			}

	//    //Downsample published cloud
	//    cloud_voxel_grid.setLeafSize (0.2, 0.2, 0.2);
	//    PointCloud::Ptr cloud_ptr 			(new PointCloud);
	//    PointCloud::Ptr cloud_filtered_ptr 	(new PointCloud);
	//    *cloud_ptr = occupancy_marker;
	//    cloud_voxel_grid.setInputCloud (cloud_ptr);
	//    cloud_voxel_grid.filter (*cloud_filtered_ptr);
	//    occupancy_marker = *cloud_filtered_ptr;

	occupancy_marker_pub_.publish(occupancy_marker);
}
/*
octomap_msgs::OcTree *ThetaStar3D::getMap()
{
	return m;
}
*/
bool ThetaStar3D::setInitialPosition(DiscretePosition p_)
{
	if (isInside(p_.x, p_.y, p_.z))
	{
		ThetaStartNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(
			p_.x,
			p_.y,
			p_.z)];

		if (initialNodeInWorld->node == NULL)
		{
			initialNodeInWorld->node = new ThetaStarNode3D(use_astar);
			initialNodeInWorld->node->point.x = p_.x;
			initialNodeInWorld->node->point.x = p_.y;
			initialNodeInWorld->node->point.x = p_.z;

			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
		}
		disc_initial = initialNodeInWorld->node;

		initial_position.x = p_.x * step;
		initial_position.y = p_.y * step;
		initial_position.z = p_.z * step;
		disc_initial->point = p_;
		disc_initial->parentNode = disc_initial;

		return true;
	}
	else
	{
		//~ std::cerr << "ThetaStar: Initial point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
		disc_initial = NULL;
		return false;
	}
}

bool ThetaStar3D::setInitialPosition(Vector3 p)
{
	initial_position = p;
	DiscretePosition p_ = discretizePosition(p);

	if (isInside(p_.x, p_.y, p_.z))
	{
		ThetaStartNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(
			p_.x,
			p_.y,
			p_.z)];

		if (initialNodeInWorld->node == NULL)
		{
			initialNodeInWorld->node = new ThetaStarNode3D(use_astar);
			initialNodeInWorld->node->point.x = p_.x;
			initialNodeInWorld->node->point.x = p_.y;
			initialNodeInWorld->node->point.x = p_.z;

			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
		}
		disc_initial = initialNodeInWorld->node;

		disc_initial->point = p_;
		disc_initial->parentNode = disc_initial;

		return true;
	}
	else
	{
		//ROS_ERROR("ThetaStar: Initial point [%f, %f, %f] not valid.", p.x, p.y, p.z);
		disc_initial = NULL;
		return false;
	}
}

bool ThetaStar3D::setFinalPosition(DiscretePosition p_)
{
	if (isInside(p_.x, p_.y, p_.z))
	{
		ThetaStartNodeLink3D *finalNodeInWorld = &discrete_world[getWorldIndex(
			p_.x,
			p_.y,
			p_.z)];

		if (finalNodeInWorld->node == NULL)
		{
			finalNodeInWorld->node = new ThetaStarNode3D(use_astar);
			finalNodeInWorld->node->point.x = p_.x;
			finalNodeInWorld->node->point.x = p_.y;
			finalNodeInWorld->node->point.x = p_.z;

			finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
		}
		disc_final = finalNodeInWorld->node;

		final_position.x = p_.x * step;
		final_position.y = p_.y * step;
		final_position.z = p_.z * step;
		disc_final->point = p_;

		return true;
	}
	else
	{
		//~ std::cerr << "ThetaStar: Final point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
		disc_final = NULL;
		return false;
	}
}

bool ThetaStar3D::setFinalPosition(Vector3 p)
{
	DiscretePosition p_ = discretizePosition(p);

	if (isInside(p_.x, p_.y, p_.z))
	{
		ThetaStartNodeLink3D *finalNodeInWorld = &discrete_world[getWorldIndex(
			p_.x,
			p_.y,
			p_.z)];

		if (finalNodeInWorld->node == NULL)
		{
			finalNodeInWorld->node = new ThetaStarNode3D(use_astar);
			finalNodeInWorld->node->point.x = p_.x;
			finalNodeInWorld->node->point.y = p_.y;
			finalNodeInWorld->node->point.z = p_.z;

			finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
		}
		disc_final = finalNodeInWorld->node;

		final_position = p;
		disc_final->point = p_;

		return true;
	}
	else
	{

		//~ std::cerr << "ThetaStar: Final point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
		disc_final = NULL;
		return false;
	}
}

Vector3 ThetaStar3D::getInitialPosition()
{
	return initial_position;
}

Vector3 ThetaStar3D::getFinalPosition()
{
	return final_position;
}

bool ThetaStar3D::lineofsight(ThetaStarNode3D &p1, ThetaStarNode3D &p2)
{
	//printf("Checking LofS: [%d, %d, %d] to [%d, %d, %d]\n", p1.point.x, p1.point.y, p1.point.z, p2.point.x, p2.point.y, p2.point.z);

	//min distance for no collision, in discrete space
	float min_distance = 5.0 * step; // 1.9; //sqrt(8);
	int extra_cells = 0;			 // min(2.0f,min_distance);

	//distance doble triangle.
	int x0 = max(min(p1.point.x, p2.point.x) - extra_cells, ws_x_min);
	int x1 = min(max(p1.point.x, p2.point.x) + extra_cells, ws_x_max);
	int y0 = max(min(p1.point.y, p2.point.y) - extra_cells, ws_y_min);
	int y1 = min(max(p1.point.y, p2.point.y) + extra_cells, ws_y_max);
	int z0 = max(min(p1.point.z, p2.point.z) - extra_cells, ws_z_min);
	int z1 = min(max(p1.point.z, p2.point.z) + extra_cells, ws_z_max);

	if (isOccupied(p1) || isOccupied(p2))
		return false;

    if (distanceBetween2nodes(p1, p2) > line_of_sight/step )
		return false;
	

	float base = distanceBetween2nodes(p1, p2);
	for (int x = x0; x <= x1; x++)
		for (int y = y0; y <= y1; y++)
			for (int z = z0; z <= z1; z++)
			{
				//If the point is occupied, we have to calculate distance to the line.
				if (!discrete_world[getWorldIndex(x, y, z)].notOccupied)
				{
					//If base is zero and node is occcupied, directly does not exist line of sight
					if (base == 0)
					{
#ifdef SEND_NO_LOFS_NODES_MARKERS
						geometry_msgs::Point p;
						p.x = p2.point.x * step;
						p.y = p2.point.y * step;
						p.z = p2.point.z * step;
						marker_no_los.header.stamp = ros::Time();
						marker_no_los.header.seq++;
						marker_no_los.points.push_back(p);
						no_los_marker_pub_.publish(marker_no_los);
#endif

						return false;
					}

					//cout << "Cell is occupied" << endl;
					float a = sqrt(pow(x - p1.point.x, 2) + pow(p1.point.y - y, 2) + pow(p1.point.z - z, 2));
					float c = sqrt(pow(p2.point.x - x, 2) + pow(p2.point.y - y, 2) + pow(p2.point.z - z, 2));
					float l = (pow(base, 2) + pow(a, 2) - pow(c, 2)) / (2 * base);
					float distance = sqrt(abs(pow(a, 2) - pow(l, 2)));

					if (distance <= min_distance)
					{
#ifdef SEND_NO_LOFS_NODES_MARKERS
						geometry_msgs::Point p;
						p.x = p2.point.x * step;
						p.y = p2.point.y * step;
						p.z = p2.point.z * step;
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

bool ThetaStar3D::isOccupied(ThetaStarNode3D n)
{
	return !discrete_world[getWorldIndex(n.point.x, n.point.y, n.point.z)].notOccupied;
}

bool ThetaStar3D::isInitialPositionOccupied()
{
	if (isOccupied(*disc_initial))
		return true;
	else
		return false;
}

bool ThetaStar3D::isFinalPositionOccupied()
{
	if (isOccupied(*disc_final))
		return true;
	else
		return false;
}

float ThetaStar3D::getMapResolution()
{
	return step;
}

DiscretePosition ThetaStar3D::discretizePosition(Vector3 p)
{
	DiscretePosition res;

	res.x = p.x * step_inv;
	res.y = p.y * step_inv;
	res.z = p.z * step_inv;

	return res;
}

bool ThetaStar3D::searchInitialPosition2d(float maxDistance)
{
	DiscretePosition init_ = discretizePosition(initial_position); // Start coordinates

	int maxDistance_ = maxDistance / step; // celdas mas lejanas a compromabar a maxDistance meters

	// Check final point, first if is inside the workspace and second if is occupied
	if (setValidInitialPosition(init_))
		return true;

	// Check from z=0 from the near xy ring to the far xy ring
	for (int d = 1; d <= maxDistance_; d++)
	{
		if (searchInitialPositionInXyRing(init_.x, init_.y, init_.z, d))
			return true;
	}

	return false;
}

bool ThetaStar3D::searchInitialPosition3d(float maxDistance)
{
	DiscretePosition init_ = discretizePosition(initial_position); // Start coordinates

	int maxDistance_ = maxDistance / step; // celdas mas lejanas a 1 metro

	// Check init point, first if is inside the workspace and second if is occupied
	if (setValidInitialPosition(init_))
		return true;

	// Check from z=0 to d from the near xy ring to the far xy ring
	for (int d = 1; d <= maxDistance_; d++)
	{
		for (int z = init_.z; z <= init_.z + d; z++)
		{
			if (searchInitialPositionInXyRing(init_.x, init_.y, z, d - (z - init_.z)))
				return true;
		}
	}

	return false;
}

bool ThetaStar3D::searchInitialPosition3dBack(float maxDistance)
{
	DiscretePosition init_ = discretizePosition(initial_position); // Start coordinates

	int maxDistance_ = maxDistance / step; // celdas mas lejanas a compromabar a maxDistance meters

	// Check init point, first if is inside the workspace and second if is occupied
	if (setValidInitialPosition(init_))
		return true;

	// Check from z=0 to d from the near xy ring to the far xy ring
	for (int d = 1; d <= maxDistance_; d++)
	{
		for (int z = init_.z; z <= init_.z + d; z++)
		{
			if (searchInitialPositionInXyRingBack(init_.x, init_.y, z, d - (z - init_.z)))
				return true;
		}
	}

	return false;
}

bool ThetaStar3D::searchFinalPosition2d(float maxDistance)
{
	DiscretePosition final_ = discretizePosition(final_position); // Start coordinates

	int maxDistance_ = maxDistance / step; // celdas mas lejanas a compromabar a maxDistance meters

	// Check final point, first if is inside the workspace and second if is occupied
	if (setValidFinalPosition(final_))
		return true;

	// Check from z=0 from the near xy ring to the far xy ring
	for (int d = 1; d <= maxDistance_; d++)
	{
		if (searchFinalPositionInXyRing(final_.x, final_.y, final_.z, d))
			return true;
	}

	return false;
}

bool ThetaStar3D::searchFinalPosition3d(float maxDistance)
{
	DiscretePosition final_ = discretizePosition(final_position); // Start coordinates

	int maxDistance_ = maxDistance / step; // celdas mas lejanas a compromabar a maxDistance meters

	// Check final point, first if is inside the workspace and second if is occupied
	if (setValidFinalPosition(final_))
		return true;

	// Check from z=0 to d from the near xy ring to the far xy ring
	for (int d = 1; d <= maxDistance_; d++)
	{
		for (int z = final_.z; z <= final_.z + d; z++)
		{
			if (searchFinalPositionInXyRing(final_.x, final_.y, z, d - (z - final_.z)))
				return true;
		}
	}

	return false;
}

bool ThetaStar3D::searchFinalPosition3dAhead(float maxDistance)
{
	DiscretePosition final_ = discretizePosition(final_position); // Start coordinates

	int maxDistance_ = maxDistance / step; // celdas mas lejanas a compromabar a maxDistance meters

	// Check final point, first if is inside the workspace and second if is occupied
	if (setValidFinalPosition(final_))
		return true;

	// Check from z=0 to d from the near xy ring to the far xy ring
	for (int d = 1; d <= maxDistance_; d++)
	{
		for (int z = final_.z; z <= final_.z + d; z++)
		{
			if (searchFinalPositionInXyRingAhead(final_.x, final_.y, z, d - (z - final_.z)))
				return true;
		}
	}

	return false;
}

bool ThetaStar3D::searchFinalPosition3dAheadHorizontalPrior(float maxDistance)
{
	DiscretePosition final_ = discretizePosition(final_position); // Start coordinates

	int maxDistance_ = maxDistance / step; // celdas mas lejanas a compromabar a maxDistance meters

	// Check final point, first if is inside the workspace and second if is occupied
	if (setValidFinalPosition(final_))
		return true;

	// Check from z=0 to d from the near xy ring to the far xy ring
	for (int d = 1; d <= maxDistance_; d++)
	{
		for (int z = final_.z; z <= final_.z + d; z++)
		{
			if (searchFinalPositionInXyRingAheadHorPriority(final_.x, final_.y, z, d - (z - final_.z)))
				return true;
		}
	}

	return false;
}

void ThetaStar3D::setTimeOut(int sec)
{
	timeout = sec;
}

int ThetaStar3D::getTimeOut()
{
	return timeout;
}
void ThetaStar3D::computeAStarPath(){

	ros::Time last_time_ = ros::Time::now();
	bool noSolution = false;
	long iter = 0;
	ThetaStarNode3D *current;
	while(!open.empty()){
		iter++;

#ifdef SEND_EXPLORED_NODES_MARKERS
			publishMarker(*current, false);
			//usleep(20000);
#endif

#ifdef PRINT_EXPLORED_NODES_NUMBER
			expanded_nodes_number_++;
#endif

		if (iter % 100 == 0)
		{
			if ((ros::Time::now() - last_time_).toSec() > timeout)
			{
				noSolution = true;
				std::cerr << "Theta Star: Timeout. Iteractions:" << iter << std::endl;
			}
		}
		auto current_it = open.begin();
		current = *current_it;
		open.erase(current_it);
		
		std::cout<< "Current point: [" << current->point.x << ", " << current->point.y << ", " << current->point.z << "]" << 
					" Parent: [" << current->parentNode->point.x << ", "                         << current->parentNode->point.y                         << ", " << current->parentNode->point.z << "]" << 
					" Parent: [" << current->parentNode->parentNode->point.x << ", "             << current->parentNode->parentNode->point.y             << ", " << current->parentNode->parentNode->point.z << "]" << 
					" Parent: [" << current->parentNode->parentNode->parentNode->point.x << ", " << current->parentNode->parentNode->parentNode->point.y << ", " << current->parentNode->parentNode->parentNode->point.z << "]" << std::endl;
		candidates.insert(current);
		
		if( !((*current) != (*disc_final))){//Solution found
			std::cout<< "Solution found!" << std::endl;
			noSolution = false;
			break;
		}
		
		set<ThetaStarNode3D *, NodePointerComparator3D> neighbors;
		getNeighbors(*current, neighbors);
		// std::cout << "Neighboors size: " << neighbors.size() << std::endl;
		for(auto &it: neighbors){//Calculated neigbors does not produce collisions
			
			auto cost = current->totalDistance;
			// std::cout << "Current cost: " << current->totalDistance << std::endl;
			ThetaStarNode3D *successor = findNodeOnList(open, it);
			ThetaStarNode3D *successor2 = findNodeOnList(candidates, it);
			if(successor == nullptr && successor2 == nullptr){
				successor = new ThetaStarNode3D(use_astar);
				successor = it;
				successor->parentNode = current;
				successor->lineDistanceToFinalPoint = weightedDistanceToGoal(*successor);
				successor->totalDistance = weightedDistanceFromInitialPoint(*successor,*successor->parentNode) + successor->lineDistanceToFinalPoint;
				open.insert(successor);
			}else if( successor2 == nullptr && cost < successor->totalDistance){
				successor->parentNode = current;
				successor->totalDistance = cost;
			}
		}	
	}
			
	

// #ifdef SEND_EXPLORED_NODES_MARKERS
// 	publishMarker(*min_distance, true);
// #endif

#ifdef PRINT_EXPLORED_NODES_NUMBER
	ROS_INFO("Theta Star: Expanded nodes: %d", expanded_nodes_number_);
#endif

	//Path finished, get final path
	last_path.clear();
	ThetaStarNode3D *path_point;
	Vector3 point;

	path_point = current;
	if (noSolution)
	{
		std::cerr << "Imposible to calculate a solution" << std::endl;
	}
	ROS_INFO("Disc initial: [%f, %f, %f]", disc_initial->point.x*step,disc_initial->point.y*step,disc_initial->point.z*step);

	while (!noSolution && path_point != disc_initial)
	{
		point.x = path_point->point.x * step;
		point.y = path_point->point.y * step;
		point.z = path_point->point.z * step;
		/*ROS_INFO("Inserting [%f, %f, %f]", point.x,point.y,point.z);
		ROS_INFO("\tParent is [%f, %f, %f]", path_point->parentNode->point.x*step,path_point->parentNode->point.y*step,path_point->parentNode->point.z*step);
		ROS_INFO("\t\ttParent is [%f, %f, %f]", path_point->parentNode->parentNode->point.x*step,path_point->parentNode->parentNode->point.y*step,path_point->parentNode->parentNode->point.z*step);
		ROS_INFO("\t\t\tParent is [%f, %f, %f]", path_point->parentNode->parentNode->parentNode->point.x*step,path_point->parentNode->parentNode->parentNode->point.y*step,path_point->parentNode->parentNode->parentNode->point.z*step);
		*/
		last_path.insert(last_path.begin(), point);

		path_point = path_point->parentNode;
		if (!path_point || (path_point == path_point->parentNode && path_point != disc_initial))
		{
			last_path.clear();
			break;
		}
#ifdef STEP_BY_STEP
	usleep(2e5);
#endif
	}

}
ThetaStarNode3D* ThetaStar3D::findNodeOnList(set<ThetaStarNode3D *, NodePointerComparator3D> &_set, ThetaStarNode3D *node){
	for(auto &it: _set){
		if( ! (*it != *node) ){
			// std::cout << "Node found in open list!" << std::endl;
			return node;
		}
	}
	return nullptr;
}
void ThetaStar3D::computeLazyThetaStarPath(){
	
	ThetaStarNode3D *min_distance = disc_initial; // s : current node
	bool noSolution = false;
	long iter = 0;
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
			expanded_nodes_number_++;
#endif
			open.erase(open.begin());
			min_distance->nodeInWorld->isInOpenList = false;

			//Insert it in candidate list
			candidates.insert(min_distance);
			min_distance->nodeInWorld->isInCandidateList = true;

			//Look for Neighbors with line of sight
			set<ThetaStarNode3D *, NodePointerComparator3D> neighbors;
			getNeighbors(*min_distance, neighbors);

			//Check if exist line of sight.
			SetVertex(*min_distance, neighbors);

			set<ThetaStarNode3D *, NodePointerComparator3D>::iterator it_;
			it_ = neighbors.begin();
			ThetaStarNode3D *new_node;
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

// #ifdef SEND_EXPLORED_NODES_MARKERS
// 	publishMarker(*min_distance, true);
// #endif

#ifdef PRINT_EXPLORED_NODES_NUMBER
	ROS_INFO("Theta Star: Expanded nodes: %d", expanded_nodes_number_);
#endif

	//Path finished, get final path
	last_path.clear();
	ThetaStarNode3D *path_point;
	Vector3 point;

	path_point = min_distance;
	if (noSolution)
	{
		std::cerr << "Imposible to calculate a solution" << std::endl;
	}
	ROS_INFO("Disc initial: [%f, %f, %f]", disc_initial->point.x*step,disc_initial->point.y*step,disc_initial->point.z*step);

	while (!noSolution && path_point != disc_initial)
	{
		point.x = path_point->point.x * step;
		point.y = path_point->point.y * step;
		point.z = path_point->point.z * step;
		//ROS_INFO("Inserting [%f, %f, %f]", point.x,point.y,point.z);
		//ROS_INFO("Parent is [%f, %f, %f]", path_point->parentNode->point.x*step,path_point->parentNode->point.y*step,path_point->parentNode->point.z*step);

		last_path.insert(last_path.begin(), point);

		path_point = path_point->parentNode;
		if (!path_point || (path_point == path_point->parentNode && path_point != disc_initial))
		{
			last_path.clear();
			break;
		}
	}
}
int ThetaStar3D::computePath(void)
{
	//~ printf("Calculating...\n");
	struct timeb start, finish;
    float seconds, milliseconds;

 	ftime(&start);
	if (disc_initial == NULL || disc_final == NULL)
	{
		std::cerr << "ThetaStar: Cannot calculate path. Initial or Final point not valid." << std::endl;
		return 0;
	}

	marker.points.clear();
	marker_no_los.points.clear();
	geometry_msgs::Point p;
	p.x = disc_initial->point.x * step;
	p.y = disc_initial->point.y * step;
	p.z = disc_initial->point.z * step;
	marker.points.push_back(p);

	//Initialize data structure. --> clear lists
	ThetaStarNode3D *erase_node;
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

	open.insert(disc_initial);
	disc_initial->nodeInWorld->isInOpenList = true;

	//Inicialize loop
	// ThetaStarNode3D *min_distance = disc_initial; // s : current node
	bool noSolution = false;
	// long iter = 0;

	if (isOccupied(*disc_initial) || isOccupied(*disc_final))
	{
		noSolution = true;
		std::cerr << "ThetaStar: Initial or Final point not free." << std::endl;
	}

#ifdef PRINT_EXPLORED_NODES_NUMBER
	expanded_nodes_number_ = 0;
#endif

	if(use_astar && !noSolution){
		computeAStarPath();
	}else if(!noSolution){
		computeLazyThetaStarPath();
	}
    ftime(&finish);

	if(!noSolution && last_path.size() != 0 && save_data_){

    	seconds = finish.time - start.time - 1;
    	milliseconds = (1000 - start.millitm) + finish.millitm;
		double time_spent =  milliseconds + seconds * 1000;
		// out_file_data_ << "AStar "<< ", " << "Robot Radius" << ", " << "Cost Scaling Factor" << ", " << 
		//                   "Cost Weight" << ", " << "Line of sight" << ", " << "Number of points" << ", " << 
		//					 "Path lenght" << ", " << "Nodos explorados" << "Tiempo" <<  std::endl;
		float x, y, z;
    	double pathLength = 0;
    	for (int i = 0; i < last_path.size()-1; i++)
    	{
    	    x = last_path.at(i).x - last_path.at(i+1).x;
    	    y = last_path.at(i).y - last_path.at(i+1).y;
    	    z = last_path.at(i).z - last_path.at(i+1).z;
    	    pathLength += sqrtf(x * x + y * y + z * z);
    	}

		std::vector<float> closest_distances;
		for(auto &it: last_path)
			closest_distances.push_back( getClosestObstacle(it));
		
		auto minmax = std::minmax_element(closest_distances.begin()+1, closest_distances.end()-1);
		float min   = std::numeric_limits<float>::max();
		float max  = std::numeric_limits<float>::max();

		if(minmax.first != closest_distances.end()) 
			min = *minmax.first;
		
		if(minmax.second != closest_distances.end()) 
			max = *minmax.second;

		auto init_closest = *closest_distances.begin();
		auto final_closest = *closest_distances.end();
		
		out_file_data_ << std::boolalpha << use_astar     << ", " << robot_radius_          << ", " << cost_scaling_factor_ << ", " << 
		                  				cost_weight       << ", " << line_of_sight          << ", " << last_path.size()     << ", " << 
										pathLength        << ", " << expanded_nodes_number_ << ", " << time_spent           <<  "," << 
										init_closest      << ","  << final_closest          << ", " << min << ", " <<
										max << std::endl;
	}
	return last_path.size();
}

vector<Vector3> ThetaStar3D::getCurrentPath()
{
	return last_path;
}

bool ThetaStar3D::getTrajectoryYawFixed(Trajectory &trajectory, double fixed_yaw)
{
	if (!trajectoryParamsConfigured)
		return false;

	// number of PATH points without the initial point
	int n_path = last_path.size();

	// Time respect first path/trajectory position (0.0)
	double total_time = 0.0;

	// Trajectory point to fill the complete trajectory msg
	TrajectoryPoint trajectory_point;
	trajectory_point.transforms.resize(1);
	trajectory_point.velocities.resize(1);
	trajectory_point.accelerations.resize(1);

	// loop last and next path positions
	Vector3 last_position = initial_position;
	Vector3 next_position;

	// loop middle trajectory position (needed if next position is so far)
	Vector3 middle_position;

	// Flags
	bool isFirst = true;	   // to set a different velocity from initial position to the first position
	bool pathPointGot = false; // to know if a middle point is necessary

	// trajectory times
	double dt_h, dt_v, dt;

	// entire path loop
	int i = 0;
	ROS_INFO("n_path: %d", n_path);
	while (i < n_path)
	{
		// get next path position
		next_position = last_path[i];
		// ROS_INFO("Next Position: [%.2f, %.2f, %.2f]", next_position.x, next_position.y, next_position.z);
		// trajectory middle waypoints
		middle_position = last_position;
		// ROS_INFO("Middle Position: [%.2f, %.2f, %.2f]", middle_position.x, middle_position.y, middle_position.z);

		// two path points loop
		pathPointGot = false;
		while (!pathPointGot)
		{
			// Check if it's neccesary a middle wp between last and next wps
			pathPointGot = checkMiddlePosition(last_position, next_position, middle_position, dxyz_tolerance);

			// Set the intermediate position if path point has not been got
			if (!pathPointGot)
			{
				//.. to the trajectory
				// calculate neccesary time to get the horizontal and vertical position
				if (isFirst)
				{
					dt_h = getHorizontalNorm(middle_position.x - last_position.x, middle_position.y - last_position.y) / vm_xy_1;
					dt_v = fabs(middle_position.z - last_position.z) / vm_z_1;
					isFirst = false;
				}
				else
				{
					dt_h = getHorizontalNorm(middle_position.x - last_position.x, middle_position.y - last_position.y) / vm_xy;
					dt_v = fabs(middle_position.z - last_position.z) / vm_z;
				}

				// Set the maximum neccesary elapse time
				dt = max(dt_h, dt_v);
				total_time += dt;

				// Set path position (last_position is the original last_position or the last middle_position if it exists)
				setPositionYawAndTime(trajectory_point, middle_position, fixed_yaw, total_time);
				trajectory.points.push_back(trajectory_point);

				//.. to the algorihtm
				last_position = middle_position;
			}
		}

		// Set path position, always as Vm_1 (last_position is the original last_position or the last middle_position if it exists)
		dt_h = getHorizontalNorm(next_position.x - last_position.x, next_position.y - last_position.y) / vm_xy_1;
		dt_v = fabs(next_position.z - last_position.z) / vm_z_1;

		// Set the maximum neccesary elapse time
		dt = max(dt_h, dt_v);
		total_time += dt;

		setPositionYawAndTime(trajectory_point, next_position, fixed_yaw, total_time);
		trajectory.points.push_back(trajectory_point);

		// next path position
		isFirst = true;
		last_position = next_position;
		i++;
	}

	//print actual trajectory vector state
	printfTrajectory(trajectory, "Trajectory_state_1");

	return true;
}

bool ThetaStar3D::getTrajectoryYawAtTime(Trajectory &trajectory, Transform init_pose)
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
	double dt_y = 0.0;
	double time_inc = 0.0; // time increment if yaw reference changes

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
		dt_y = dyaw / w_yaw;
		last_yaw = yaw;
		q.setRPY(0.0, 0.0, yaw);
		trajectory.points[k].transforms[0].rotation.x = q.x();
		trajectory.points[k].transforms[0].rotation.y = q.y();
		trajectory.points[k].transforms[0].rotation.z = q.z();
		trajectory.points[k].transforms[0].rotation.w = q.w();

		// calculate time increment
		if (k == 0)
			time_inc = dt_y - trajectory.points[k].time_from_start.toSec();
		else
			time_inc = dt_y - (trajectory.points[k].time_from_start.toSec() - trajectory.points[k - 1].time_from_start.toSec());

		// check if neccesary time for the turn in yaw is bigger than for the position one
		if (time_inc > 0.0)
		{
			// update time for this waypoint and all next waypoints
			for (int j = k; j < trajectory.points.size(); j++)
				trajectory.points[j].time_from_start += ros::Duration(time_inc);

			//ROS_INFO("[%d] Exist time increment: %f",  k, time_inc);
		}
	}

	//print actual trajectory vector state
	//printfTrajectory(trajectory, "Trajectory_state_2");

	return true;
}

bool ThetaStar3D::getTrajectoryYawInAdvance(Trajectory &trajectory, Transform init_pose)
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
	double dt_y = 0.0;
	int i = 0;
	printf("traj_size = %d\n", traj_size);
	while (getHorizontalNorm(trajectory.points[i].transforms[0].translation.x - initial_position.x, trajectory.points[i].transforms[0].translation.y - initial_position.y) < min_yaw_ahead)
	{
		printf("Intial Pos: [%.4f, %.4f], Tajectory Point: [%.4f,%.4f]\n", initial_position.x, initial_position.y, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y);
		i++;

		if (i > traj_size - 1)
		{
			printf("break\n");
			i = -1;
			break;
		}
	}
	if (i >= 0)
	{
		yaw = atan2(trajectory.points[i].transforms[0].translation.y - initial_position.y, trajectory.points[i].transforms[0].translation.x - initial_position.x);
		printf("i=%d, Setting yaw = %.4f from [%.4f, %.4f] to [%.4f, %.4f]\n", i, yaw, initial_position.x, initial_position.y, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.x);
		dyaw = getDyaw(yaw, yaw_odom);
		dt_y = dyaw / w_yaw;
		last_yaw = yaw;
		setPositionYawAndTime(trajectory_point, initial_position, yaw, dt_y);
		trajectory.points.insert(trajectory.points.begin(), trajectory_point);

		// update time for all next waypoints
		if (dt_y > 0.0)
			for (int j = 1; j < trajectory.points.size(); j++)
				trajectory.points[j].time_from_start += ros::Duration(dt_y);
	}

	//print actual trajectory vector state
	//printfTrajectory(trajectory, "Trajectory_state_1");

	// Transform the previous trajectory setting the Yaw ahead in Advance and the neccesay time increment for get it
	tf::Quaternion q;	  // yaw as quaternion to the msg
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
		dt_y = dyaw / w_yaw;
		last_yaw = yaw;
		// set as quaternion
		q.setRPY(0.0, 0.0, yaw);
		trajectory.points[k].transforms[0].rotation.x = q.x();
		trajectory.points[k].transforms[0].rotation.y = q.y();
		trajectory.points[k].transforms[0].rotation.z = q.z();
		trajectory.points[k].transforms[0].rotation.w = q.w();
		// calculate time increment
		time_inc = dt_y - (trajectory.points[k].time_from_start.toSec() - trajectory.points[k - 1].time_from_start.toSec());
		// check if neccesary yaw turn time is bigger than for the position increment
		if (time_inc > 0.0)
		{
			//ROS_INFO("[%d] Exist time increment: %f",  k, time_inc);
			// update time for this waypoint and all next waypoints
			for (int j = k; j < trajectory.points.size(); j++)
			{
				trajectory.points[j].time_from_start += ros::Duration(time_inc);
			}
			//printfTrajectory(trajectory, "Trajectory_state_middle");
		}
	}
	// last waypoint yaw reference (same that second last, so yaw does not change, so time does not change)
	q.setRPY(0.0, 0.0, yaw); // Note: If it's only one wp the 'for' statement does not execute nothing, so its neccesary this line
	trajectory.points.back().transforms[0].rotation.x = q.x();
	trajectory.points.back().transforms[0].rotation.y = q.y();
	trajectory.points.back().transforms[0].rotation.z = q.z();
	trajectory.points.back().transforms[0].rotation.w = q.w();

	//print actual trajectory vector state
	//printfTrajectory(trajectory, "Trajectory_state_2");

	return true;
}

bool ThetaStar3D::getTrajectoryYawInAdvanceWithFinalYaw(Trajectory &trajectory, Transform init_pose, double final_yaw_ref)
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
			double pre_last_time = trajectory.points.at(traj_size - 2).time_from_start.toSec();
			dyaw = getDyaw(final_yaw_ref, pre_last_yaw);
			double current_dt = trajectory.points.back().time_from_start.toSec() - pre_last_time;
			double new_dt = dyaw / w_yaw;
			if (new_dt > current_dt)
			{
				// Set new time
				trajectory.points.back().time_from_start = ros::Duration(new_dt + pre_last_time);
			}
		}
	}
}

void ThetaStar3D::getNeighbors(ThetaStarNode3D &node, set<ThetaStarNode3D *, NodePointerComparator3D> &neighbors)
{
	neighbors.clear();

	DiscretePosition node_temp;

	for (int i = -1; i < 2; i++)
	{
		for (int j = -1; j < 2; j++)
		{
			for (int k = -1; k < 2; k++)
			{
				/*
                 *Ignore ourselves
                */
				if (i != 0 || j != 0 || k != 0)
				{
					node_temp.x = node.point.x + i;
					node_temp.y = node.point.y + j;
					node_temp.z = node.point.z + k;

					if (isInside(node_temp.x, node_temp.y, node_temp.z))
					{
						int nodeInWorld = getWorldIndex(node_temp.x, node_temp.y, node_temp.z);

						ThetaStartNodeLink3D *new_neighbor = &discrete_world[nodeInWorld];

						if (new_neighbor->node == NULL)
						{
							new_neighbor->node = new ThetaStarNode3D(use_astar);
							new_neighbor->node->point.x = node_temp.x;
							new_neighbor->node->point.y = node_temp.y;
							new_neighbor->node->point.z = node_temp.z;
							new_neighbor->node->nodeInWorld = new_neighbor;
							new_neighbor->node->parentNode = &node;
						}
						if(use_astar && new_neighbor->notOccupied && !new_neighbor->isInCandidateList){
							neighbors.insert(new_neighbor->node);
							continue;
						}
						if (!use_astar && new_neighbor->isInCandidateList || lineofsight(node, *new_neighbor->node))
						{
							neighbors.insert(new_neighbor->node);
						}
					}
					else
					{
						// delete new_neighbor;
					}
				}
			}
		}
	}
}

void ThetaStar3D::publishMarker(ThetaStarNode3D &s, bool publish)
{
	geometry_msgs::Point point;
	point.x = s.point.x * step;
	point.y = s.point.y * step;
	point.z = s.point.z * step;

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
	usleep(2e5);
#endif
}

float ThetaStar3D::distanceToGoal(ThetaStarNode3D node)
{
	return sqrt(pow(disc_final->point.x - node.point.x, 2) +
				pow(disc_final->point.y - node.point.y, 2) +
				pow(disc_final->point.z - node.point.z, 2));
}

float ThetaStar3D::weightedDistanceToGoal(ThetaStarNode3D node)
{
	return goal_weight * sqrt(pow(disc_final->point.x - node.point.x, 2) +
							  pow(disc_final->point.y - node.point.y, 2) +
							  z_weight_cost * pow(disc_final->point.z - node.point.z, 2));
}

float ThetaStar3D::distanceBetween2nodes(ThetaStarNode3D &n1, ThetaStarNode3D &n2)
{
	return sqrt(pow(n1.point.x - n2.point.x, 2) +
				pow(n1.point.y - n2.point.y, 2) +
				pow(n1.point.z - n2.point.z, 2));
}
//!Aqui hay que meter el coste
float ThetaStar3D::weightedDistanceBetween2nodes(ThetaStarNode3D &n1, ThetaStarNode3D &n2)
{
	double cost = m_grid3d->getProbabilityFromPoint(n2.point.x*step, n2.point.y*step, n2.point.z*step);
	// cout<<"Prob: "<<cost<<endl;
	// ROS_INFO("Node: [%f, %f, %f, %lf], Cost w: %f", n2.point.x*step,n2.point.y*step, n2.point.z*step, cost, cost_weight);
	
	return cost * cost_weight + (sqrt(pow(n1.point.x - n2.point.x, 2) +
				pow(n1.point.y - n2.point.y, 2) +
				z_weight_cost * pow(n1.point.z - n2.point.z, 2)));
}
float ThetaStar3D::distanceFromInitialPoint(ThetaStarNode3D node, ThetaStarNode3D parent)
{
	float res;
	if (isOccupied(node))
		res = std::numeric_limits<float>::max();
	else if (parent.distanceFromInitialPoint == std::numeric_limits<float>::max())
		res = parent.distanceFromInitialPoint;
	else
	{
		res = parent.distanceFromInitialPoint + (sqrt(pow(node.point.x - parent.point.x, 2) +
													  pow(node.point.y - parent.point.y, 2) +
													  pow(node.point.z - parent.point.z, 2)));
	}

	return res;
}
//!Aqui tambien hay que meter el coste
float ThetaStar3D::weightedDistanceFromInitialPoint(ThetaStarNode3D node, ThetaStarNode3D parent)
{
	float res;
	double cost = m_grid3d->getProbabilityFromPoint(node.point.x*step, node.point.y*step, node.point.z*step);

	// double cost = m_grid3d->getProbabilityFromPoint(node.point.x, node.point.y, node.point.z);

	if (isOccupied(node))
		res = std::numeric_limits<float>::max();
	else if (parent.distanceFromInitialPoint == std::numeric_limits<float>::max())
		res = parent.distanceFromInitialPoint;
	else
	{
		res = parent.distanceFromInitialPoint + cost * cost_weight + (sqrt(pow(node.point.x - parent.point.x, 2) +
													  pow(node.point.y - parent.point.y, 2) +
													  z_weight_cost * pow(node.point.z - parent.point.z, 2)));
	}

	return res;
}

void ThetaStar3D::ComputeCost(ThetaStarNode3D &s, ThetaStarNode3D &s2)
{
	double distanceParent2 = weightedDistanceBetween2nodes((*s.parentNode), s2);
	if (s.parentNode->distanceFromInitialPoint + distanceParent2 + s2.lineDistanceToFinalPoint < s2.totalDistance)
	{
		s2.parentNode = s.parentNode;
		s2.distanceFromInitialPoint = weightedDistanceFromInitialPoint(s2, *s2.parentNode);
		s2.totalDistance = s2.distanceFromInitialPoint + s2.lineDistanceToFinalPoint;
	}
}

void ThetaStar3D::UpdateVertex(ThetaStarNode3D &s, ThetaStarNode3D &s2)
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

void ThetaStar3D::SetVertex(ThetaStarNode3D &s, set<ThetaStarNode3D *, NodePointerComparator3D> &neighbors)
{
	if (!lineofsight(*s.parentNode, s))
	{
		float g_value = std::numeric_limits<float>::max();
		ThetaStarNode3D *parentCandidate = NULL;
		set<ThetaStarNode3D *, NodePointerComparator3D>::iterator it_;
		it_ = neighbors.begin();
		ThetaStarNode3D *new_node;
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

double ThetaStar3D::g(ThetaStarNode3D &s)
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

void ThetaStar3D::setPositionYawAndTime(TrajectoryPoint &trajectory_point, Vector3 position, double _yaw, double T)
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
	trajectory_point.time_from_start = ros::Duration(T);
}

bool ThetaStar3D::checkMiddlePosition(Vector3 last_position, Vector3 next_position, Vector3 &middle_position, float tolerance)
{
	// Flag
	bool pathPointGot = true; // to know if a middle point is necessary (funtion returned value)
	bool Limited_V = false;   // to know if a the next point is limited vertically, to check correctly the horizontal limitation

	// Position max increments (where descomposes dxy_max)
	double DXmax, DYmax;

	// Check Vertically Limit
	if (fabs(next_position.z - last_position.z) > (dz_max + dxyz_tolerance))
	{
		// Z to the max
		if (next_position.z - last_position.z > 0.0)
			middle_position.z = last_position.z + dz_max;
		else
			middle_position.z = last_position.z - dz_max;

		// XY proportional to the Z
		double alfa = dz_max / fabs(next_position.z - last_position.z); // Porcentaje del recorrido que se va a realizar hasta esta Z limitada
		middle_position.x = last_position.x + alfa * (next_position.x - last_position.x);
		middle_position.y = last_position.y + alfa * (next_position.y - last_position.y);

		// Exist limitation, so the point is not directly reached
		pathPointGot = false;
		Limited_V = true;
	}

	// Check Horizontal Limit
	/* WARNING: it is necessary to distinguish between if Z has been limited or not: 
		 *	- If not: The current X Y target are the next position in the path --> the next_position 
		 *	- If yes: The current X Y target are the proportional to the Z limited --> the current middle_position */

	if (!Limited_V && getHorizontalNorm(next_position.x - last_position.x, next_position.y - last_position.y) > (dxy_max + dxyz_tolerance))
	{
		// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
		if (fabs(next_position.x - last_position.x) >= 0.001)
		{
			DXmax = dxy_max / (sqrt(1.0 + pow(next_position.y - last_position.y, 2) / pow(next_position.x - last_position.x, 2)));
			DYmax = fabs(next_position.y - last_position.y) / fabs(next_position.x - last_position.x) * DXmax;
		}
		else
		{
			DXmax = 0.0;
			DYmax = dxy_max;
		}

		if (next_position.x - last_position.x > 0.0)
			middle_position.x = last_position.x + DXmax;
		else
			middle_position.x = last_position.x - DXmax;

		if (next_position.y - last_position.y > 0.0)
			middle_position.y = last_position.y + DYmax;
		else
			middle_position.y = last_position.y - DYmax;

		// Z proportional to the XY limited
		double beta = dxy_max / getHorizontalNorm(next_position.x - last_position.x, next_position.y - last_position.y);
		middle_position.z = last_position.z + beta * (next_position.z - last_position.z);

		// Exist limitation, so the point is not directly reached
		pathPointGot = false;
	}
	else if (Limited_V && getHorizontalNorm(middle_position.x - last_position.x, middle_position.y - last_position.y) > (dxy_max + dxyz_tolerance))
	{
		// X and Y to the max (It exists a singularity at DX = 0.0, so if that directly set the known values)
		if (fabs(middle_position.x - last_position.x) >= 0.001)
		{
			DXmax = dxy_max / (sqrt(1.0 + pow(middle_position.y - last_position.y, 2) / pow(middle_position.x - last_position.x, 2)));
			DYmax = fabs(middle_position.y - last_position.y) / fabs(middle_position.x - last_position.x) * DXmax;
		}
		else
		{
			DXmax = 0.0;
			DYmax = dxy_max;
		}

		// Z proportional to the XY limited (it need to be do previous to modificate middle_position.x and .y)
		double beta = dxy_max / getHorizontalNorm(middle_position.x - last_position.x, middle_position.y - last_position.y);
		middle_position.z = last_position.z + beta * (middle_position.z - last_position.z);

		if (next_position.x - last_position.x > 0.0)
			middle_position.x = last_position.x + DXmax;
		else
			middle_position.x = last_position.x - DXmax;

		if (next_position.y - last_position.y > 0.0)
			middle_position.y = last_position.y + DYmax;
		else
			middle_position.y = last_position.y - DYmax;

		// Exist limitation, so the point is not directly reached
		pathPointGot = false;
	}

	return pathPointGot;
}

double ThetaStar3D::getHorizontalNorm(double x, double y)
{
	return sqrtf(x * x + y * y);
}

double ThetaStar3D::getHorizontalDistance(TrajectoryPoint P1, TrajectoryPoint P2)
{
	return getHorizontalNorm(P1.transforms[0].translation.x - P2.transforms[0].translation.x, P1.transforms[0].translation.y - P2.transforms[0].translation.y);
}

float ThetaStar3D::getYawFromQuat(Quaternion quat)
{
	double r, p, y;
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 M(q);
	M.getRPY(r, p, y);

	return y;
}

double ThetaStar3D::getDyaw(double next_yaw, double last_yaw)
{
	double dyaw = fabs(next_yaw - last_yaw);
	if (dyaw > M_PI)
		dyaw = fabs(dyaw - 2.0 * M_PI);

	return dyaw;
}

void ThetaStar3D::printfTrajectory(Trajectory trajectory, string trajectory_name)
{
	printf(PRINTF_YELLOW "%s trajectory [%d]:\n", trajectory_name.c_str(), (int)trajectory.points.size());
	std::vector<float> closest_distances;

	for (unsigned int i = 0; i < trajectory.points.size(); i++)
	{
		double yaw = getYawFromQuat(trajectory.points[i].transforms[0].rotation);
		printf(PRINTF_BLUE "\t %d: [%f, %f, %f] m\t[%f] rad\t [%f] sec\n", i, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y, trajectory.points[i].transforms[0].translation.z, yaw, trajectory.points[i].time_from_start.toSec());
		closest_distances.push_back( getClosestObstacle(trajectory.points[i].transforms[0].translation));
	}
	std_msgs::Float32MultiArray msg; 
	msg.data = closest_distances;
	closest_distances_pub_.publish(msg);
	std_msgs::Float32 msg_min;
	msg_min.data = 1000000;
	for(auto &it: closest_distances){
		if(it < msg_min.data)
			msg_min.data = it;
	}
	closest_distance_pub_.publish(msg_min);
	printf(PRINTF_REGULAR);
}
double ThetaStar3D::getClosestObstacle(const geometry_msgs::Vector3 &positon){

	double prob = m_grid3d->getProbabilityFromPoint(positon.x, positon.y, positon.z);
	//Inverse the probabilty function to get dist:
	//! prob = 100*exp(-cost_scaling_factor*std::fabs((dist - robot_radius)));
	if(cost_scaling_factor_ == 0){
		ROS_ERROR("You should set cost scaling factor to calculate obstacle dist to a value different than 0");
		return 0;
	}
	double closest = std::fabs((std::log(prob / 100 ) / cost_scaling_factor_ ) )+ robot_radius_;
	return closest;
}
bool ThetaStar3D::setCostsParamsSrv(theta_star_2d::SetCostParams::Request& _req, theta_star_2d::SetCostParams::Response& _rep ){

	_rep.success = true;
	cost_scaling_factor_ = _req.cost_scaling_factor;
	robot_radius_ = _req.robot_radius;
	_rep.message = "Cost scaling factor set to " + std::to_string(cost_scaling_factor_) + ", Robot radius set to " + std::to_string(robot_radius_);
	return true;
}
bool ThetaStar3D::switchAstar(std_srvs::Trigger::Request &_req, std_srvs::Trigger::Response &_rep){
	
	if(use_astar){
		use_astar = false;
	}else{
		goal_weight = 1.0; //No goal weight
		z_weight_cost = 1.0;
		cost_weight = 0.0;
		std::cout<<"Using A* Algorithm" << std::endl;
		use_astar  = true;
	}
	_rep.success = true;
	return true;
}
void ThetaStar3D::setMinObstacleRadius(double minR_){
	minR=minR_;
}

void ThetaStar3D::confPrintRosWarn(bool print)
{
	PRINT_WARNINGS = print;
}
void ThetaStar3D::set3DCostWeight(double cost){
	if(use_astar)
		return;
	cost_weight = cost;
}
void ThetaStar3D::set3DMaxLineOfSightDist(double dist){
	if(use_astar)
		return;

	line_of_sight = dist;
}

} // namespace PathPlanners
