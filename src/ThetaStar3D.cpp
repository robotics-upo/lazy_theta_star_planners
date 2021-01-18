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
//#define PRINT_EXPLORED_NODES_NUMBER
// Uncomment to get the explored nodes (at time or slowly step_by_step)
#define SEND_EXPLORED_NODES_MARKERS
//#define STEP_BY_STEP
// Uncomment to get non-LineOfSight visual markers
#define SEND_NO_LOFS_NODES_MARKERS
// Uncomment to printf octree leaf free and occupied
//#define PRINT_OCTREE_STATS
// Uncomment to printf if setVertex() fails at no-LofS
//#define PRINT_SETVERTEX_FAILS
// Uncomment to set length catenary in nodes
#define USE_CATENARY_COMPUTE

//*****************************************************************
// 				ThetaStar Algorithm Class Definitions
//*****************************************************************
// Default constructor
ThetaStar3D::ThetaStar3D()
{
}

// Constructor with arguments
ThetaStar3D::ThetaStar3D(std::string plannerName, std::string frame_id, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_, float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, ros::NodeHandlePtr n)
{
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

	topicPath = plannerName + "/vis_marker_occupancy_reduced";
	marker_reduced_pub_ = nh->advertise<RVizMarker>(topicPath.c_str(), 1);
	marker_reduced.header.frame_id = frame_id; //"world";
	marker_reduced.header.stamp = ros::Time();
	marker_reduced.ns = "debug";
	marker_reduced.id = 66;
	marker_reduced.type = RVizMarker::CUBE_LIST;
	marker_reduced.action = RVizMarker::ADD;
	marker_reduced.pose.orientation.w = 1.0;
	marker_reduced.scale.x = 1.0 * step;
	marker_reduced.scale.y = 1.0 * step;
	marker_reduced.scale.z = 1.0 * step;
	marker_reduced.color.a = 1.0;
	marker_reduced.color.r = 1.0;
	marker_reduced.color.g = 1.0;
	marker_reduced.color.b = 1.0;
	
}

ThetaStar3D::~ThetaStar3D()
{
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
			dist=sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
			if(dist < minR){
				discrete_world[world_index_].notOccupied = true;
				continue;
			}
			else
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

octomap::OcTree ThetaStar3D::updateMapReduced(octomap_msgs::OctomapConstPtr msg, 
									geometry_msgs::Vector3Stamped goal_, 
									geometry_msgs::Vector3Stamped start_, 
									geometry_msgs::Vector3 rpy_,
									std::vector<octomap::point3d> &full_ray_cast, 
									std::vector<octomap::point3d> &ray_cast_free,
									std::vector<octomap::point3d> &ray_cast_free_reduce,  
									std::vector<octomap::point3d> &ray_cast_coll, 
									std::vector<octomap::point3d> &no_ray_cast_free)
{
	full_ray_cast.clear(); ray_cast_free.clear(); ray_cast_free_reduce.clear(); ray_cast_coll.clear(); no_ray_cast_free.clear();

	// geometry_msgs::Vector3 new_start, new_goal;
	double offset_start_ = 0.25;
	double offset_goal_ = 0.25;
	double _resolution = 0.2;
	double R_, theta_, phi_;

	new_start.x = start_.vector.x + 0.5 * ((start_.vector.x - goal_.vector.x) / sqrt(pow(start_.vector.x - goal_.vector.x,2)+pow(start_.vector.y - goal_.vector.y,2)+pow(start_.vector.z - goal_.vector.z,2)));
	new_start.y = start_.vector.y + 0.5 * ((start_.vector.y - goal_.vector.y) / sqrt(pow(start_.vector.x - goal_.vector.x,2)+pow(start_.vector.y - goal_.vector.y,2)+pow(start_.vector.z - goal_.vector.z,2)));
	new_start.z = start_.vector.z + 0.5 * ((start_.vector.z - goal_.vector.z) / sqrt(pow(start_.vector.x - goal_.vector.x,2)+pow(start_.vector.y - goal_.vector.y,2)+pow(start_.vector.z - goal_.vector.z,2)));
	new_goal.x = goal_.vector.x + 0.5 * ((goal_.vector.x - start_.vector.x) / sqrt(pow(goal_.vector.x - start_.vector.x,2)+pow(goal_.vector.y - start_.vector.y,2)+pow(goal_.vector.z - start_.vector.z,2)));
	new_goal.y = goal_.vector.y + 0.5 * ((goal_.vector.y - start_.vector.y) / sqrt(pow(goal_.vector.x - start_.vector.x,2)+pow(goal_.vector.y - start_.vector.y,2)+pow(goal_.vector.z - start_.vector.z,2)));
	new_goal.z = goal_.vector.z + 0.5 * ((goal_.vector.z - start_.vector.z) / sqrt(pow(goal_.vector.x - start_.vector.x,2)+pow(goal_.vector.y - start_.vector.y,2)+pow(goal_.vector.z - start_.vector.z,2)));


	std::vector<octomap::point3d> vrc_, points_rcnf_, points_rcf_;
	std::vector<octomap::point3d> map_simplify_free, map_simplify_occupied;
	vrc_.clear(); points_rcnf_.clear();  points_rcf_.clear(); map_simplify_free.clear(); map_simplify_occupied.clear();
	octomap::point3d s_(new_start.x, new_start.y, new_start.z);
	octomap::point3d g_(new_goal.x, new_goal.y, new_goal.z);
	octomap::point3d r_, e_;
	octomap::OcTree *map_msg;
	octomap::OcTree map_simplify (0.05f);
	octomap::OcTree map_simplify_reduced (0.05f);
	bool r_cast_coll;

	map_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);

	double angle_ = M_PI/8;
	double hip_ = sqrt(pow(new_goal.x-new_start.x,2)+ pow(new_goal.y-new_start.y,2)+ pow(new_goal.z-new_start.z,2));
	double sweep_range = hip_ * sin(angle_); 
	offset_area = 0.5;
	phi_min = -1.0*sweep_range - offset_area;	
	phi_max = sweep_range + offset_area;
	theta_min = -1.0* sweep_range - offset_area;
	theta_max = sweep_range + offset_area;
	double z_max_obs = 0.0; 
	double max_z_to_explore = -100.0;

	octomap::point3d Vgs(new_goal.x-new_start.x, new_goal.y-new_start.y, new_goal.z-new_start.z);

	//Get spherical parameter
	R_= sqrt(Vgs.x()*Vgs.x() + Vgs.y()*Vgs.y() + Vgs.z()*Vgs.z());
	theta_ = atan(sqrt(Vgs.x()*Vgs.x()+Vgs.y()*Vgs.y())/(Vgs.z()));
	phi_ = atan(Vgs.y()/Vgs.x());
	//Transform cartesian coordinates to spherical vectorial base 
	float ur_x,ur_y,ur_z,ut_x,ut_y,ut_z,up_x,up_y,up_z;
	ur_x = sin(theta_) * cos(phi_);
	ur_y = sin(theta_) * sin(phi_);
	ur_z = cos(theta_);
	ut_x = cos(theta_) * cos(phi_);
	ut_y = cos(theta_) * sin(phi_);
	ut_z = -sin(theta_);
	up_x= -sin(phi_);
	up_y = cos(phi_);
	up_z = 0;
	Eigen::Matrix3f base_sp;
	base_sp << ur_x, ur_y, ur_z, 
			   ut_x, ut_y, ut_z, 
			   up_x, up_y, up_z;
    Eigen::Vector3f plane_sp;
	
	//1. Applying Ray Cast to detect 3D free collision space and with collision. The space where Ray cast has collision is used to set highest obstacle and reduce 3D space to set with full Ray 
	int count_total_full = 0;
	int count_ray_free = 0;
	int count_ray_collision = 0;
	int count_ray_free_reduced = 0;

	for (double delta_theta = theta_min ; delta_theta <= theta_max; delta_theta = delta_theta + _resolution){
		for (double delta_phi = phi_min ; delta_phi <= phi_max; delta_phi = delta_phi + _resolution){	
			plane_sp << 0.0, 
						delta_theta, 
						delta_phi;
			Eigen::Matrix3f base_sp_inv =  base_sp.inverse();
			Eigen::Vector3f spheric_ = base_sp_inv * plane_sp;
			Eigen::Vector3f cartesian_;
			cartesian_ << spheric_(0,0)+new_goal.x,
						  spheric_(1,0)+new_goal.y, 
						  spheric_(2,0)+new_goal.z;

			octomap::point3d e_(cartesian_(0,0), cartesian_(1,0), cartesian_(2,0)); //current end(goal) point
			octomap::point3d d_(e_.x() - new_start.x , e_.y() - new_start.y , e_.z() - new_start.z ); //direction for rayCast

			r_cast_coll = map_msg->castRay(s_, d_, r_);

			map_msg->computeRay(s_, e_, vrc_);
			count_total_full++;	
			for (size_t i = 0; i < vrc_.size(); i ++){
				full_ray_cast.push_back(vrc_[i]);
			}
			
			if(!r_cast_coll){
				map_msg->computeRay(s_, e_, vrc_);
				for (size_t i = 0; i < vrc_.size(); i ++){
					ray_cast_free.push_back(vrc_[i]);
				}
				count_ray_free++;
				points_rcf_.push_back(e_);
			}
			else{
				count_ray_collision ++;
				double d1_ = pow(new_start.x - r_.x(),2) + pow(new_start.y - r_.y(),2) + pow(new_start.z - r_.z(),2);
				double d2_ = pow(new_start.x - e_.x(),2) + pow(new_start.y - e_.y(),2) + pow(new_start.z - e_.z(),2);
				if (d1_ >= d2_ ){
					map_msg->computeRay(s_, e_, vrc_);
					points_rcf_.push_back(e_);
				}
				else{
					map_msg->computeRay(s_, r_, vrc_);
					points_rcnf_.push_back(e_);
				}
				if (z_max_obs < r_.z()){
					z_max_obs = r_.z();
					max_z_to_explore = e_.z();
				}
				for (size_t i = 0; i < vrc_.size(); i ++){
					ray_cast_coll.push_back(vrc_[i]);
					if (i < vrc_.size()-1)
						map_simplify_free.push_back(vrc_[i]);
					else
						map_simplify_occupied.push_back(vrc_[i]);
				}

			}
		}
	}
	printf("------------------ COUNT_RAY_FULL = [%i]\n",count_total_full);
	printf("------------------ COUNT_RAY_FREE = [%i]\n",count_ray_free);
	printf("------------------ COUNT_RAY_COLLISION = [%i]\n",count_ray_collision);


	//2. Apply Full Ray to set full 3D space for exploration  and reduced free space above highest collision 
	for (size_t i= 0 ; i < points_rcf_.size(); i++){
		octomap::point3d e_(points_rcf_[i].x(), points_rcf_[i].y(), points_rcf_[i].z()); //save the current end(goal) point
		map_msg->computeRay(s_, e_, vrc_);
		if (points_rcf_[i].z() < max_z_to_explore- _resolution){	
			count_ray_free_reduced++;
			for (size_t i = 0; i < vrc_.size(); i ++){
				ray_cast_free_reduce.push_back(vrc_[i]);
				map_simplify_free.push_back(vrc_[i]);
			}
		}
	}
	printf("------------------ COUNT_RAY_FREE_REDUCED = [%i]\n",count_ray_free_reduced);
	printf("------------------ SIZE  points_rcf_ = [%lu] max_z_to_explore=[%f]\n",points_rcf_.size(), max_z_to_explore);

	//3. Process to get rayCast in opposite direction, from goal zone to start zone to fill the collision zone
	int count_ray_inverse = 0;
	int count_ray_inverse_if = 0; 
	int count_ray_inverse_else = 0; 
	for (size_t i = 0; i < points_rcnf_.size(); i ++){
		count_ray_inverse++;
		if (points_rcnf_[i].z() < max_z_to_explore- _resolution){
			octomap::point3d d_((new_start.x-points_rcnf_[i].x()) ,(new_start.y - points_rcnf_[i].y()) ,(new_start.z - points_rcnf_[i].z()) ); //save the direction for rayCast
			octomap::point3d c_;
			map_msg->castRay(points_rcnf_[i], d_, c_);
			double d1_ = pow(new_start.x - points_rcnf_[i].x(),2) + pow(new_start.y - points_rcnf_[i].y(),2) + pow(new_start.z - points_rcnf_[i].z(),2);
			double d2_ = pow(c_.x() - points_rcnf_[i].x(),2) + pow(c_.y() - points_rcnf_[i].y(),2) + pow(c_.z() - points_rcnf_[i].z(),2);
			if (d2_ >= d1_ ){
				count_ray_inverse_if++;
				octomap::point3d new_s_(new_start.x,new_start.y,new_start.z);
				map_msg->computeRay(points_rcnf_[i], new_s_, vrc_);
			}
			else{
				count_ray_inverse_else++;
				map_msg->computeRay(points_rcnf_[i], c_, vrc_);
			}
			for (size_t j = 0; j < vrc_.size(); j++){
				no_ray_cast_free.push_back(vrc_[j]);
				if (j < vrc_.size()-1){
					map_simplify_free.push_back(vrc_[j]);
				}
				else{
					map_simplify_occupied.push_back(vrc_[j]);
				}
			}
		}	
	}
	printf("------------------ COUNT_RAY_INVERSE = [%i]\n",count_ray_inverse);
	printf("------------------ COUNT_RAY_INVERSE_IF = [%i]  COUNT_RAY_INVERSE_ELSE = [%i]\n",count_ray_inverse_if, count_ray_inverse_else);
	printf("------------------ SIZE  points_rcnf_ = [%lu] \n",points_rcnf_.size());


	//4. Create new octomap with desired space to navigate
	// insert some measurements of occupied cells
	for (size_t i=0 ; i < map_simplify_occupied.size() ; i++){
		map_simplify.updateNode(map_simplify_occupied[i], true); // integrate 'occupied' measurement
		// publishMarkerReduced(map_simplify_occupied[i],true);
	}
  	// insert some measurements of free cells
	for (size_t i=0 ; i < map_simplify_free.size() ; i++){
		map_simplify.updateNode(map_simplify_free[i], false);  // integrate 'free' measurement
		// publishMarkerReduced(map_simplify_free[i],true);
	}
	
	u_int64_t occupied_leafs = 0, free_leafs = 0;
	int nit = 0;

	std::vector<int> v_disc_x_, v_disc_y_, v_disc_z_;
	std::vector<int> v_disc_free_x_, v_disc_free_y_, v_disc_free_z_;
	v_disc_x_.clear(); v_disc_y_.clear(); v_disc_z_.clear();
	v_disc_free_x_.clear(), v_disc_free_y_.clear(), v_disc_free_z_.clear();
	bool match_;
	int x_, y_ , z_;
	float x_w, y_w, z_w;
	// Read from first to the last leaf of the tree set its xyz (discretized) if it is occupied into the occupancy matrix

	float step_red = 0.1;
	float step_inv_red = 1.0/0.1;

	if (map_simplify.begin_leafs() != NULL)
	{
		for (octomap::OcTree::leaf_iterator it = map_simplify.begin_leafs(), end = map_simplify.end_leafs(); it != end; ++it)
		{
			// // Get occupied cells
			// x_w = it.getX();
			// y_w = it.getY();
			// z_w = it.getZ();

			// // Exact discretization
			// x_ = (int)(x_w * step_inv_red);
			// y_ = (int)(y_w * step_inv_red);
			// z_ = (int)(z_w * step_inv_red);

			if (map_simplify.isNodeOccupied(*it))
			{
				// Get occupied cells
				x_w = it.getX();
				y_w = it.getY();
				z_w = it.getZ();

				// Exact discretization
				x_ = (int)(x_w * step_inv_red);
				y_ = (int)(y_w * step_inv_red);
				z_ = (int)(z_w * step_inv_red);

				if(v_disc_x_.size() < 1){
					v_disc_x_.push_back(x_);
					v_disc_y_.push_back(y_);
					v_disc_z_.push_back(z_);
				}
				else{
					match_ = false;
					for(size_t i=0 ; i<v_disc_x_.size() ; i++){
						if (v_disc_x_[i] == x_){
							if(v_disc_y_[i] == y_){
								if(v_disc_z_[i] == z_){
									match_ = true;
									break;	
								}
							}
						}
					}
					if (match_ != true){
						v_disc_x_.push_back(x_);
						v_disc_y_.push_back(y_);
						v_disc_z_.push_back(z_);
					}
				}
			}
			// else
			// {
			// 	if(v_disc_free_x_.size() < 1){
			// 		v_disc_free_x_.push_back(x_);
			// 		v_disc_free_y_.push_back(y_);
			// 		v_disc_free_z_.push_back(z_);
			// 	}
			// 	else{
			// 		match_ = false;
			// 		for(size_t i=0 ; i<v_disc_free_x_.size() ; i++){
			// 			if (v_disc_free_x_[i] == x_){
			// 				if(v_disc_free_y_[i] == y_){
			// 					if(v_disc_free_z_[i] == z_){
			// 						match_ = true;
			// 						break;	
			// 					}
			// 				}
			// 			}
			// 		}
			// 		if (match_ != true){
			// 			v_disc_free_x_.push_back(x_);
			// 			v_disc_free_y_.push_back(y_);
			// 			v_disc_free_z_.push_back(z_);
			// 		}
			// 	}
			// }
		}
	}
	printf("=================== size v_disc_x_=[%lu] \n",v_disc_x_.size());
	clearMapReduced(v_disc_x_.size());
	// Set as occupied in the discrete matrix reduced
	for(size_t i = 0 ; i < v_disc_x_.size() ; i++){
		unsigned int world_index_ = nit;
		++nit;
		discrete_world_reduced[world_index_].notOccupied = false;
		octomap::point3d reduced_points_(v_disc_x_[i]* step_red,v_disc_y_[i]* step_red,v_disc_z_[i]* step_red);
		map_simplify_reduced.updateNode(reduced_points_, true);

		if (h_inflation >= step || v_inflation >= step)
		{
			if (z_w > z_not_inflate)
				inflateNodeAsXyRectangle(x_, y_, z_);
			else
				inflateNodeAsCylinder(x_, y_, z_);
		}
	}
	// for(size_t i = 0 ; i < v_disc_free_x_.size() ; i++){
	// 	octomap::point3d reduced_points_(v_disc_free_x_[i]* step_red, v_disc_free_y_[i]* step_red,v_disc_free_z_[i]* step_red);
	// 	map_simplify_reduced.updateNode(reduced_points_, true);
	// }

	return map_simplify_reduced;
}

// Clear the map
void ThetaStar3D::clearMap()
{
	for (int i = 0; i < matrix_size; i++)
	{
		discrete_world[i].notOccupied = true;
	}
}

// Clear the map
void ThetaStar3D::clearMapReduced(size_t _size)
{
	printf("clearMapReduced _size=[%lu]\n",_size);
	
	discrete_world_reduced.resize(_size);
	for (int i = 0; i < _size; i++)
	{
		discrete_world_reduced[i].notOccupied = true;
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
			initialNodeInWorld->node = new ThetaStarNode3D();
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
			initialNodeInWorld->node = new ThetaStarNode3D();
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
			finalNodeInWorld->node = new ThetaStarNode3D();
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
			finalNodeInWorld->node = new ThetaStarNode3D();
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

	geometry_msgs::Vector3 p0_;
	p0_.x= p1.point.x*step;
	p0_.y= p1.point.y*step;
	p0_.z= p1.point.z*step;
	if (use_catenary && !feasibleCatenary(p1, tf_reel,p0_) )
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

int ThetaStar3D::computePath(void)
{
	//~ printf("Calculating...\n");

	if (disc_initial == NULL || disc_final == NULL)
	{
		std::cerr << "ThetaStar: Cannot calculate path. Initial or Final point not valid." << std::endl;
		return 0;
	}

	marker.points.clear();
	marker_reduced.points.clear();
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
	ThetaStarNode3D *min_distance = disc_initial; // s : current node
	bool noSolution = false;
	long iter = 0;

	if (isOccupied(*disc_initial) || isOccupied(*disc_final))
	{
		noSolution = true;
		std::cerr << "ThetaStar: Initial or Final point not free." << std::endl;
	}

#ifdef PRINT_EXPLORED_NODES_NUMBER
	int expanded_nodes_number = 0;
#endif

	ros::Time last_time_ = ros::Time::now();

	int time_count_aux = -1;

	while (!noSolution && (*min_distance) != (*disc_final))
	{
		int time_count = floor((ros::Time::now() - last_time_).toSec());
		if (time_count!= time_count_aux){
			printf("Time compute Global Path: %i sec.\n",time_count);
			time_count_aux  = time_count ;
		}

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

#ifdef SEND_EXPLORED_NODES_MARKERS
	publishMarker(*min_distance, true);
	publishOccupationMarkersMap();
#endif

#ifdef PRINT_EXPLORED_NODES_NUMBER
	ROS_INFO("Theta Star: Expanded nodes: %d", expanded_nodes_number);
#endif

	//Path finished, get final path
	last_path.clear();
#ifdef USE_CATENARY_COMPUTE	
	length_catenary.clear();
#endif
	ThetaStarNode3D *path_point;
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
		point.z = path_point->point.z * step;
		double length_ = path_point->lengthCatenary;
		// if (length_ <=0.0){
		// 	if(feasibleCatenary(*path_point,tf_reel)){
		// 		length_ = path_point->lengthCatenary;
		// 	}
		// }
		printf("point=[%f %f %f] path_point->point=[%i %i %i] length_=[%f]\n",point.x,point.y,point.z,path_point->point.x,path_point->point.y,path_point->point.z,length_);

		last_path.insert(last_path.begin(), point);
#ifdef USE_CATENARY_COMPUTE	
		length_catenary.insert(length_catenary.begin(), length_);
#endif
		path_point = path_point->parentNode;
		if (!path_point || (path_point == path_point->parentNode && path_point != disc_initial))
		{
			last_path.clear();
			length_catenary.clear();
			break;
		}
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
		ROS_INFO("Next Position: [%.2f, %.2f, %.2f]", next_position.x, next_position.y, next_position.z);
		// trajectory middle waypoints
		middle_position = last_position;
		ROS_INFO("Middle Position: [%.2f, %.2f, %.2f]", middle_position.x, middle_position.y, middle_position.z);

		int count_j_ = 0;
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

#ifdef USE_CATENARY_COMPUTE
				double lengthToset;
				length_catenary_aux.clear();
				for(size_t j=0 ; j < length_catenary.size(); j++){
					length_catenary_aux.push_back(length_catenary[j]) ;
				}
				length_catenary.clear();

				ThetaStarNode3D init_wp, midd_wp;
				// midd_wp.point.x= middle_position.x*step_inv;
				// midd_wp.point.y= middle_position.y*step_inv;
				// midd_wp.point.z= middle_position.z*step_inv;
				midd_wp.point.x= trajectory_point.transforms[0].translation.x*step_inv;
				midd_wp.point.y= trajectory_point.transforms[0].translation.y*step_inv;
				midd_wp.point.z= trajectory_point.transforms[0].translation.z*step_inv;
				if(feasibleCatenary(midd_wp,tf_reel,trajectory_point.transforms[0].translation))
					lengthToset = midd_wp.lengthCatenary;
				// printf("valor de i=[%i]midd_wp.point=[%i %i %i][%f %f %f]  tf_reel=[%f %f %f] length=[%f] step_inv=[%f]\n",
				// i,midd_wp.point.x,midd_wp.point.y,midd_wp.point.z,trajectory_point.transforms[0].translation.x,trajectory_point.transforms[0].translation.y,trajectory_point.transforms[0].translation.z,
				// tf_reel.x,tf_reel.y,tf_reel.z,lengthToset,step_inv);

				for (size_t j = 0 ; j < length_catenary_aux.size(); j++){
					if (count_j_==j)
						length_catenary.push_back(lengthToset);
					length_catenary.push_back(length_catenary_aux[j]);
				}
#endif 			
				count_j_++;
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
							new_neighbor->node = new ThetaStarNode3D();
							new_neighbor->node->point.x = node_temp.x;
							new_neighbor->node->point.y = node_temp.y;
							new_neighbor->node->point.z = node_temp.z;
							new_neighbor->node->nodeInWorld = new_neighbor;
							new_neighbor->node->parentNode = &node;
						}

						if (new_neighbor->isInCandidateList || lineofsight(node, *new_neighbor->node))
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

void ThetaStar3D::getNeighborsReduced(ThetaStarNode3D &node, set<ThetaStarNode3D *, NodePointerComparator3D> &neighbors)
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
							new_neighbor->node = new ThetaStarNode3D();
							new_neighbor->node->point.x = node_temp.x;
							new_neighbor->node->point.y = node_temp.y;
							new_neighbor->node->point.z = node_temp.z;
							new_neighbor->node->nodeInWorld = new_neighbor;
							new_neighbor->node->parentNode = &node;
						}

						if (new_neighbor->isInCandidateList || lineofsight(node, *new_neighbor->node))
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
	usleep(1e4);
#endif
}

void ThetaStar3D::publishMarkerReduced(octomap::point3d _p, bool publish)
{
	geometry_msgs::Point _point;
	_point.x = _p.x();
	_point.y = _p.y();
	_point.z = _p.z();

	marker_reduced.points.push_back(_point);

#ifdef STEP_BY_STEP
	publish = true;
#endif

	if (publish)
	{
		marker_reduced.header.stamp = ros::Time();
		marker_reduced.header.seq++;
		marker_reduced_pub_.publish(marker_reduced);
	}

#ifdef STEP_BY_STEP
	usleep(1e4);
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

float ThetaStar3D::weightedDistanceBetween2nodes(ThetaStarNode3D &n1, ThetaStarNode3D &n2)
{
	return sqrt(pow(n1.point.x - n2.point.x, 2) +
				pow(n1.point.y - n2.point.y, 2) +
				z_weight_cost * pow(n1.point.z - n2.point.z, 2));
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

float ThetaStar3D::weightedDistanceFromInitialPoint(ThetaStarNode3D node, ThetaStarNode3D parent)
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

	for (unsigned int i = 0; i < trajectory.points.size(); i++)
	{
		double yaw = getYawFromQuat(trajectory.points[i].transforms[0].rotation);
		printf(PRINTF_BLUE "\t %d: [%f, %f, %f] m\t[%f] rad\t [%f] sec\n", i, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y, trajectory.points[i].transforms[0].translation.z, yaw, trajectory.points[i].time_from_start.toSec());
	}

	printf(PRINTF_REGULAR);
}
void ThetaStar3D::setMinObstacleRadius(double minR_){
	minR=minR_;
}

void ThetaStar3D::confPrintRosWarn(bool print)
{
	PRINT_WARNINGS = print;
}

bool ThetaStar3D::feasibleCatenary(ThetaStarNode3D &_p1, geometry_msgs::Vector3 _p2 , geometry_msgs::Vector3 _p0)
{
	bool _p_cat_occupied= true;
	std::vector<geometry_msgs::Point> _points_catenary;
	geometry_msgs::Vector3 _p1_new;
	DiscretePosition _p_occ;
	
	double _dist_X, _dist_Y, _dist_Z,_dist;
	double _mF = 0.01;

	_dist_X = (_p0.x - _p2.x);
	_dist_Y = (_p0.y - _p2.y);
	_dist_Z = (_p0.z - _p2.z);
	_dist = sqrt(_dist_X * _dist_X + _dist_Y * _dist_Y + _dist_Z * _dist_Z);
	bool _get_catenary = false;
	int _count = 0;
	int count_coll_;
	int best_count_coll_ = 1000;
	double best_length_;
	while (!_get_catenary){
		
		double _length = _dist + _mF;

		if(_length > 15 ){
			// printf("WARNING: Not posible to get catery for point=[%f %f %f] in Theta Star because length > dist_max. Looking for a new way point\n", _p1_new.x,_p1_new.y,_p1_new.z);
			return false;
		}

		_points_catenary.clear();

		biCat.setNumberPointsCatenary(_length*10.0);
		biCat.setFactorBisection(bound_bisection_a,bound_bisection_b);
		biCat.configBisection(_length, _p2.x, _p2.y, _p2.z,_p0.x,_p0.y,_p0.z,_count,"cat_theta_star");
		biCat.getPointCatenary3D(_points_catenary);

		int _n_points_cat_dis = ceil(1.5*ceil(_length)); // parameter to ignore collsion points in the begining and in the end of catenary
		if (_n_points_cat_dis < 5)
			_n_points_cat_dis = 5;
		if (_n_points_cat_dis +1 >= _points_catenary.size())
			_n_points_cat_dis =  _points_catenary.size() -2;
		
		count_coll_ = 0;
		for (size_t j = 0 ; j <_points_catenary.size() ; j++){
			if (j > _n_points_cat_dis){
				ThetaStarNode3D _p_cat;
				_p_occ.x= _points_catenary[j].x*step_inv;
				_p_occ.y= _points_catenary[j].y*step_inv;
				_p_occ.z= _points_catenary[j].z*step_inv;
				_p_cat.point = _p_occ;

				if(_p_occ.z < ws_z_min ){
					// printf("WARNING: Not posible to get catery for point=[%f %f %f] in Theta Star because _p_occ.z < ws_z_min[%i] p_occ=[%i %i %i][%f %f %f] . Looking for a new way point\n", 
					// _p1_new.x,_p1_new.y,_p1_new.z,ws_z_min,_p_occ.x,_p_occ.y,_p_occ.z,_points_catenary[j].x,_points_catenary[j].y,_points_catenary[j].z);
					_p1.lengthCatenary = best_length_;
					return false;
				}
				_p_cat_occupied = isOccupied(_p_cat);
			
				if (_p_cat_occupied){
					// printf("ENTRO _p1_new=[%f %f %f] _p2=[%f %f %f] dist=[%f] length=[%f] point_cat=[%lu/%lu]\n",_p1_new.x,_p1_new.y,_p1_new.z,_p2.x,_p2.y,_p2.z,_dist,_length,j ,_points_catenary.size());
					if(count_coll_ == 0)
						_mF = _mF + 0.01;
					count_coll_++;
				}
			}
		}
		if(best_count_coll_ > count_coll_ && count_coll_> 0){
			best_count_coll_ = count_coll_;
			best_length_ = _length;
		}
		if(count_coll_ > 0)
			_p_cat_occupied = true;

		if (!_p_cat_occupied){
			_get_catenary = true;
			_p1.lengthCatenary = _length;
			// printf("========= ENTRO_p1_new=[%f %f %f] _p2=[%f %f %f] dist=[%f] length=[%f] point_cat=[%lu]\n",_p1_new.x,_p1_new.y,_p1_new.z,_p2.x,_p2.y,_p2.z,_dist,_length,_points_catenary.size());
			return true;
			// break
		}
		_count++;
	}
}

void ThetaStar3D::configCatenaryCompute(bool _u_c, double _mf, double _ba, double _bb, double _l_m, geometry_msgs::Vector3 _v3){
	use_catenary = _u_c;
	multiplicative_factor = _mf;
	bound_bisection_a = _ba;
	bound_bisection_b = _bb; 
	length_tether_max = _l_m;
	tf_reel.x = _v3.x;
	tf_reel.y = _v3.y;
	tf_reel.z = _v3.z;
	// printf("ThetaStar3D: VALUES:  [%d] [%f %f %f %f] [%f %f %f]!!\n",
	// use_catenary, multiplicative_factor, bound_bisection_a, bound_bisection_b, length_tether_max, tf_reel.x, tf_reel.y, tf_reel.z);
}

} // namespace PathPlanners
