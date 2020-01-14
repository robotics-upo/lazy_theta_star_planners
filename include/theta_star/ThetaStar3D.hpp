/*
 * Copyright 2016 Ricardo Ragel de la Torre, GRVC, Univ. of Seville, Spain
 *
 * Resume: 	ThetaStar Class declarations and other auxiliar classes.
 * 			Motion Path Planner based on the Theta Star algoritm: 
 * 			"Weighted Lazy Theta Star with Optimization" and different
 * 			methods to get a colission-free path [x,y,z] or 
 * 			trajectory [xyz, yaw, time].
 */

#ifndef THETASTAR3D_H_
#define THETASTAR3D_H_

#include <vector>
#include <set>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

#include <octomap_msgs/Octomap.h> //Octomap Binary
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <boost/foreach.hpp>


#define PRINTF_REGULAR  "\x1B[0m"
#define PRINTF_RED  	"\x1B[31m"
#define PRINTF_GREEN  	"\x1B[32m"
#define PRINTF_YELLOW  	"\x1B[33m"
#define PRINTF_BLUE  	"\x1B[34m"
#define PRINTF_MAGENTA  "\x1B[35m"
#define PRINTF_CYAN  	"\x1B[36m"
#define PRINTF_WHITE	"\x1B[37m"

using namespace std;

namespace PathPlanners {

typedef geometry_msgs::Transform 		Transform;
typedef geometry_msgs::Vector3 			Vector3;
typedef geometry_msgs::Quaternion 		Quaternion;
typedef pcl::PointCloud<pcl::PointXYZ> 	PointCloud;

typedef trajectory_msgs::MultiDOFJointTrajectory 		Trajectory;
typedef trajectory_msgs::MultiDOFJointTrajectoryPtr 	TrajectoryPtr;
typedef trajectory_msgs::MultiDOFJointTrajectoryPoint 	TrajectoryPoint;
typedef visualization_msgs::Marker RVizMarker;


//*****************************************************************
//				Auxiliar Class for ThetaStar Algorithm
//*****************************************************************
class DiscretePosition
{
	public:
	   int x,y,z;
};

// Nodes (declaration)
class ThetaStarNode3D;

// Links
class ThetaStartNodeLink3D
{
	public:
	   ThetaStartNodeLink3D(): 	node(NULL), isInOpenList(false), isInCandidateList(false), checked(false), 
								notOccupied(true), lastTimeSeen(std::numeric_limits<int>::max()), countToBeOccupied(0)
	   {
	   }

	   ThetaStarNode3D *node;				// Node from this link
	   bool isInOpenList;				// algorithm open list flag
	   bool isInCandidateList;			// algorithm candidate list flag
	   bool notOccupied;				// occupancy mark
		float cost;
	   // This last Three vars are not used in the ThetaStar Class.
	   // Are only for future implementations: LocalPlanner derived class
	   int lastTimeSeen;				// last time this 'node' was really seen (set as occupied) index (0 if it has been just seen)
	   unsigned char countToBeOccupied;	// number of times that this node was seen. If it is > pc_min_time_of_view, it is set as occupied
	   bool checked;					// flag to know if node (matrix element) has already been checked by other pointCloud point (case of several points of the cloud correspond to the same node)
};

// Nodes (definition)
class ThetaStarNode3D
{
	public:
	   ThetaStarNode3D():	parentNode(NULL), nodeInWorld(NULL), lineDistanceToFinalPoint( std::numeric_limits<float>::max() ), 
						distanceFromInitialPoint( std::numeric_limits<float>::max()), totalDistance(std::numeric_limits<float>::max())
	   {
	   }

	   DiscretePosition    point;		// Discrete position of this node
	   ThetaStarNode3D      *parentNode;	// pointer to parent node
	   ThetaStartNodeLink3D *nodeInWorld;	// pointer to link from this node to its parent 
	   float lineDistanceToFinalPoint;	// algorithm value
	   float distanceFromInitialPoint;	// algorithm value
	   float totalDistance;				// algorithm value

	   // Comparator '!=' definition
	   friend bool operator != (const ThetaStarNode3D& lhs, const ThetaStarNode3D& rhs)
	   {
		  return lhs.point.x!=rhs.point.x ||
				 lhs.point.y!=rhs.point.y ||
				 lhs.point.z!=rhs.point.z;
	   }
};

// Comparator for pointer to ThetaStarNode3D objects
struct NodePointerComparator3D
{
   bool operator () (const ThetaStarNode3D* const& lhs__, const ThetaStarNode3D* const& rhs__) const
   {
      const ThetaStarNode3D *lhs = lhs__;
      const ThetaStarNode3D *rhs = rhs__;
      float res = 0;

      res = lhs->totalDistance - rhs->totalDistance;
      if(res==0)
      {
         res = lhs->point.x - rhs->point.x;
      }
      if(res==0)
      {
         res = lhs->point.y - rhs->point.y;
      }
      if(res==0)
      {
         res = lhs->point.z - rhs->point.z;
      }

      if(res==0)
      {
         res = lhs->parentNode->point.x - rhs->parentNode->point.x;
      }
      if(res==0)
      {
         res = lhs->parentNode->point.y - rhs->parentNode->point.y;
      }
      if(res==0)
      {
         res = lhs->parentNode->point.z - rhs->parentNode->point.z;
      }

      return res < 0;
   }
};


//*****************************************************************
// 				ThetaStar Algoritm Class Declaration
//*****************************************************************
class ThetaStar3D
{
  public:
		/**
		  Default constructor
		**/	
		ThetaStar3D();

		/**
		  Constructor with arguments
		   @param planner name for topic names 
		   @param frame_id for debug markers 
		   @param simetric or asimetric workspace centered at (0,0,0) [meters]
		   @param occupancy matrix resolution [meters]
		   @param occupancy matrix nodes inflation (horizontal and vertical, real + safety) [meters]
		   @param Lazy Theta* with Optimization: goal point factor [0 to inf]. Bigger -> distance to target more weight than distance to origin -> minor exploration -> shorter runtime, grater path length
		   @param Lazy Theta* weighted: Z axis weight cost [0 to inf]. 0 to 1 for Z priority exploration, 1 for symetric exploration and inf(~100) to not explore in Z.
		   @param Lazy Theta* bounded: Minimum Z that will be inflated vertically 
		   @param NodeHandle 
		**/
		ThetaStar3D(std::string plannerName, std::string frame_id, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_, float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, std::shared_ptr<ros::NodeHandle> n );

		/**
		  Initialization
		   @param planner name for topic names 
		   @param frame_id for debug markers 
		   @param simetric or asimetric workspace centered at (0,0,0) [meters]
		   @param occupancy matrix resolution [meters]
		   @param occupancy matrix nodes inflation (horizontal and vertical, real + safety) [meters]
		   @param Lazy Theta* with Optimization: goal point factor [0 to inf]. Bigger -> distance to target more weight than distance to origin -> minor exploration -> shorter runtime, grater path length
		   @param Lazy Theta* weighted: Z axis weight cost [0 to inf]. 0 to 1 for Z priority exploration, 1 for symetric exploration and inf(~100) to not explore in Z.
		   @param Lazy Theta* bounded: Minimum Z that will be inflated vertically 
		   @param NodeHandle 
		**/
		void init(std::string plannerName, std::string frame_id, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_, float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, std::shared_ptr<ros::NodeHandle> n);

		/**
		  Default destructor
		**/
		~ThetaStar3D();

		/**
		 Configure the computed trajectories by getCurrentTrajectory() functions
			@param dxy_max, dz_max: Maximum increment between wps [meters]
			@param dxyz_tolerance: 	position tolerance for the path to trajectory segmentation [meters]
			@param vm_xy, vm_z: 	Mean linear velocities [m/s]
			@param vm_xy_1, vm_z_1:	Mean linear velocities for initial and final displacement [m/s]
			@param w_yaw:			Mean angular velocity at yaw [rad/s]
			@param min_yaw_ahead:	Minimum position increment to set yaw ahead [meters]
		**/
		void setTrajectoryParams(float dxy_max_, float dz_max_, float dxyz_tolerance_, float vm_xy_, float vm_z_, float vm_xy_1_, float vm_z_1_, float w_yaw_, float min_yaw_ahead_);

		/**
		  Override actual occupancy matrix
		   @param octomap msg
		**/
		void updateMap(octomap_msgs::OctomapConstPtr message);

		/**
		  Add a cloud to the actual occupancy matrix
		   @param pcl pointCloud 
		**/
		void updateMap(PointCloud cloud);
		/**
		 * 
		 * 
		**/
		void updateMap(const PointCloud::ConstPtr &map);
		/** 
		   Clear occupancy discrete matrix
		**/
		void clearMap();
		/**
		  Publish via topic the discrete map constructed
		**/
		void publishOccupationMarkersMap();

		/**
		  Returns current collision map
			@return current collision map
		**/
		octomap::OcTree * getMap();

		/**	
		  Returns map resolution
			@return map resolution
		**/
		float getMapResolution();

		/**
		  Set timeout to calculate the solution
			@param Natural number with time in seconds
		**/
		void setTimeOut(int sec);

		/**
		  Returns current timeout in seconds
			@return current timeout in seconds
		**/
		int  getTimeOut();

		/**
		  Set initial position of the path only check if 
		  it's inside WS but not if it's occupied
			@param 3D position data (Discrete or Continuous)
			@return false if is outside the workspace
		**/
		bool setInitialPosition(DiscretePosition p_);
		bool setInitialPosition(Vector3 p);

		/**
		  Set final position of the path only check if 
		  it's inside WS but not if it's occupied
			@param 3D position data (Discrete or Continuous)
			@return false if is outside the workspace
		**/
		bool setFinalPosition(DiscretePosition p_);
		bool setFinalPosition(Vector3 p);

		/**
		  Set initial/final position of the path checking if 
		  it's inside WS and if it'ts occupied (--> Valid)
		   @param [x,y,z] discrete or continuous position
		   @return true if is a valid initial/final position and has been set correctly
		**/
		inline bool setValidInitialPosition(DiscretePosition p)
		{
			if(setInitialPosition(p))
			{
				if(!isInitialPositionOccupied())	
				{
					ROS_INFO("ThetaStar: Initial discrete position [%d, %d, %d] set correctly", p.x,p.y,p.z);
					return true;
				}
			}
			else
			{
				//if(PRINT_WARNINGS)
					ROS_WARN("ThetaStar: Initial position outside the workspace attempt!!");
			}
			
			return false;
		}
		inline bool setValidInitialPosition(Vector3 p)
		{
			
			DiscretePosition pp = discretizePosition(p);
			
			return setValidInitialPosition(pp);
		}
		inline bool setValidFinalPosition(DiscretePosition p)
		{
			if(setFinalPosition(p))
			{
				if(!isFinalPositionOccupied())	
				{
					ROS_INFO("ThetaStar: Final discrete position [%d, %d, %d] set correctly", p.x,p.y,p.z);
					return true;
				}
			}
			else
			{
				//if(PRINT_WARNINGS)
					ROS_WARN("ThetaStar: Final position outside the workspace attempt!! [%d, %d, %d]", p.x,p.y,p.z);
			}
			
			return false;
		}
		inline bool setValidFinalPosition(Vector3 p)
		{
			DiscretePosition pp = discretizePosition(p);
			
			return setValidFinalPosition(pp);
		}

		/**
		 Check if the current initial position is VALID and if it
		 is not valid search and set a free initial position
		 around it.
		   @param maximum distance from initial position
		   @return true if a initial valid position has been set
		**/
		bool searchInitialPosition2d(float maxDistance); 		// Symetric search in nearest XY neighbours
		bool searchInitialPosition3d(float maxDistance);		// Symetric search in nearest XY-Zup neighbours
		bool searchInitialPosition3dBack(float maxDistance);	// Symetric search in nearest Xback-Y-Zup neighbours

		/**
		 Check if the current final position is VALID and, if it
		 is not valid, search and set a free initial position
		 around it.
		   @param maximum distance from initial position
		   @return true if a initial valid position has been set
		**/
		bool searchFinalPosition2d(float maxDistance); 						// Symetric search in nearest XY neighbours
		bool searchFinalPosition3d(float maxDistance);						// Symetric search in nearest XY-Zup neighbours
		bool searchFinalPosition3dAhead(float maxDistance);					// Symetric search in nearest XinFront-Y-Zup neighbours
		bool searchFinalPosition3dAheadHorizontalPrior(float maxDistance);	// Asymetric search in nearest XinFront-Y-Zup neighbours

		/**
		  Returns current initial/final position
			@return current initial/final position
		**/
		Vector3 getInitialPosition();
		Vector3 getFinalPosition();

		/**
		  Try to calculate a new path in the wanted timeout.			
			@return number of points in the path. 0 if no solution
		**/
		int computePath(void);

		/**
		  Get the current Path as vector [Xi Yi Zi]   
		**/
		vector<Vector3> getCurrentPath();
			
		/**
		  Get the current Trajectory from the thetaStar current path solution as 
		  trajectory_msgs::MultiDOFJointTrajectory [Xi, Yi, Zi, Yawi, ti]. 
		*/
		bool getTrajectoryYawFixed(Trajectory &trajectory, double fixed_yaw); // Simple Trayectory with fixed yaw. Times are computed by Dxyz
		bool getTrajectoryYawAtTime(Trajectory &trajectory, Transform intial_pose);	// Yaw ahead from Wp[k] to Wp[k+1] as reference from Wp[k] to Wp[k+1]. Times are computed by max(Dxyz,Dyaw)
		bool getTrajectoryYawInAdvance(Trajectory &trajectory, Transform intial_pose); // Yaw ahead from Wp[k] to Wp[k+1] as reference from Wp[k-1] to Wp[k]. Times are computed by max(Dxyz,Dyaw)
		bool getTrajectoryYawInAdvanceWithFinalYaw(Trajectory &trajectory, Transform intial_pose,  double final_yaw_ref); // Idem but it forces the last wp attitude

		/**
		 Configure if printf ROS_WARN() or not 
		**/
		void confPrintRosWarn(bool print);
			//! Aux Function: Get yaw in radians from a quaternion
		float getYawFromQuat(Quaternion quat);
		inline bool isInside(Vector3 &v)
		{
			int x_ = v.x*step_inv;
			int y_ = v.y*step_inv;
			int z_ = v.z*step_inv;
			return  isInside(x_, y_, z_);
		}
  protected:

		/**	
		  Returns the discretized position of the input 'p'
			@param Position
			@return Discretized position
		**/
		DiscretePosition discretizePosition(Vector3 p);

		/**
		 Get neighbors of a specified node
		   @param node (input)
		   @param his neighbors (output)
		**/
		void getNeighbors(ThetaStarNode3D &node, set<ThetaStarNode3D*,NodePointerComparator3D> &neighbors);

		/**
		 Returns distance to goal.
		   @param node to calculate distance from.
		   @return distance to goal.
		 Weighted version, using the 'z_weight_cost' param to increase the altitude change cost
		**/
		float distanceToGoal(ThetaStarNode3D node);
		float weightedDistanceToGoal(ThetaStarNode3D node);

		/**
		 Returns distance between 2 nodes.
		   @param origin node
		   @param target node
		   @return distance to origin to target node
		 Weighted version, using the 'z_weight_cost' param to increase the altitude change cost
		**/
		float distanceBetween2nodes(ThetaStarNode3D &n1,ThetaStarNode3D &n2);
		float weightedDistanceBetween2nodes(ThetaStarNode3D &n1,ThetaStarNode3D &n2);

		/**
		 Returns distance from initial.
		   @param node
		   @param his parent
		   @return distance to init node
		 Weighted version, using the 'z_weight_cost' param to increase the altitude change cost
		**/
		float distanceFromInitialPoint(ThetaStarNode3D node, ThetaStarNode3D parent);
		float weightedDistanceFromInitialPoint(ThetaStarNode3D node, ThetaStarNode3D parent);

		/**
		 Open and candidates node list, ordered by minor total distance (std::set --> listas ordenadas)
		**/
		set<ThetaStarNode3D*, NodePointerComparator3D> open,candidates;

		/**
		 Main algorithm functions. See http://aigamedev.com/open/tutorial/lazy-theta-star/
		**/
		void ComputeCost(ThetaStarNode3D &s, ThetaStarNode3D &s2);
		void UpdateVertex(ThetaStarNode3D &s, ThetaStarNode3D &s2);
		void SetVertex(ThetaStarNode3D &s,set<ThetaStarNode3D*,NodePointerComparator3D> &neighbors);
		double g(ThetaStarNode3D &s);

		/**	
		  Returns if exists line of sight between two points.
			@param Position of first one point
			@param Position of second one point
			@return true if exists line of sight, false in the other case.
		**/
		bool lineofsight(ThetaStarNode3D &p1, ThetaStarNode3D &p2);

		/** 
		 Check if a node is occupied
		   @param A ThetaStarNode3D
		   @return true if is occupied in the occupancy matrix
		**/
		bool isOccupied(ThetaStarNode3D n);

		/** 
		  Return true if set initial/final position is occupied
			@return true if set initial/final position is occupied
		**/
		bool isInitialPositionOccupied();
		bool isFinalPositionOccupied();

		//! Aux Function: Tajectory point to fill and send to the trajectory tracker sight ahead -- Yaw is calculated in such a way that cameras always see ahead
		void setPositionYawAndTime(TrajectoryPoint &trajectory_point, Vector3 position, double _yaw, double T);

		//! Aux Function: check if it's necessary a middle_position (waypoint) between last_position and next_position and get this middle_position 
		bool checkMiddlePosition(Vector3 last_position, Vector3 next_position, Vector3 &middle_position, float tolerance);

		//! Aux Function: return the horizonatl module of [X,Y]
		double getHorizontalNorm(double x, double y);

		//! Aux Function: return the horizontal distance from Point1 to Point2
		double getHorizontalDistance(TrajectoryPoint P1, TrajectoryPoint P2);

	
		//! Aux Function: Get differential from last_yaw to next_yaw in [-PI, PI], but in absolute value --> [0, PI] 
		double getDyaw(double next_yaw, double last_yaw);

		//! Aux Function: simply print the trajectory msg data
		void printfTrajectory(Trajectory trajectory, string trajectory_name);

		/**
		 Publish a marker in the position of a ThetaStarNode3D
		   @param node to publish his position
		   @param Set true to publish it instantly or false to simply push back at the marker array to publish later
		**/
		void publishMarker(ThetaStarNode3D &s, bool publish);


  /** Inline Functions **/

		/**
		 Get discrete occupancy matrix index for this discrete (x,y,z) position
		   @param x, y, z discrete position values
		   @return the index in the occupancy matrix
		**/
		inline unsigned int getWorldIndex(int &x, int &y, int &z)
		{
			return (unsigned int)((x - ws_x_min_inflated) + (Lx)*((y - ws_y_min_inflated) + (Ly)*(z - ws_z_min_inflated)));
		}

		/**
		 Get discrete (x,y,z) position for a discrete occupancy matrix index
		   @param x, y, z discrete position values (output)
		   @param index (input)
		**/
		inline void getDiscreteWorldPositionFromIndex(int &x, int &y, int &z, int index)
		{
			// Discrete matrix = [(ALL_X_SEGMENT)(ALL_X_SEGMENT)...(ALL_X_SEGMENT) | (ALL_X_SEGMENT)(ALL_X_SEGMENT)...(ALL_X_SEGMENT)| ............. |(ALL_X_SEGMENT)(ALL_X_SEGMENT)...(ALL_X_SEGMENT)]
			//		for			   Y=WS_Y_MIN	  Y=WS_Y_MIN+1  ... Y=WS_Y_MAX     |  Y=WS_Y_MIN	  Y=WS_Y_MIN+1  ... Y=WS_Y_MAX   | ............. | Y=WS_Y_MIN	  Y=WS_Y_MIN+1  ... Y=WS_Y_MAX    
			//		for			   				  Z = WS_Z_MIN 					   |                  Z = WS_Z_MIN+1				 | ............. |            Z = WS_Z_MAX = WS_Z_MIN+Lz      
			
			int z_n_segmts = floor( index * Lx_inv * Ly_inv );					// Index z segment (0 to Lz)
			int y_n_segmts = floor( (index - z_n_segmts * Lx * Ly) * Ly_inv );	// Index y segment (0 to Ly)
			
			x = ws_x_min_inflated + (index - z_n_segmts*Lx*Ly - y_n_segmts*Lx);
			y = ws_y_min_inflated + y_n_segmts;
			z = ws_z_min_inflated + z_n_segmts;
		}

		/**
		 Check if a Node/continuousPosition/discretePosition is inside the worksspace
		   @param ThetaStar Node / Discrete position
		   @return true if is inside the ws
		**/
		inline bool isInside(ThetaStarNode3D n)
		{
			return  isInside(n.point.x,n.point.y,n.point.z);
		}
		
		inline bool isInside(int &x, int &y, int &z)
		{
			return  (x < (ws_x_max-1) && x > (ws_x_min+1)) &&
					(y < (ws_y_max-1) && y > (ws_y_min+1)) &&
					(z < (ws_z_max-1) && z > (ws_z_min+1));
		}

		/**
		  Inflate a occupied cells filling all cells around in the occupancy matrix
		  Improvement: fast version using memset() to set to zero (.notOccupied) all 
		  occupation matrix nodes that have to be inflated
			@param discrete position of the cell to inflate		
		**/
		inline void inflateNodeAsCube(int &x_, int &y_, int &z_)
		{
			// Inflation limits around the node
			int x_inflated_max = (x_ + h_inflation) + 1;
			int x_inflated_min = (x_ - h_inflation) - 1;
			int y_inflated_max = (y_ + h_inflation) + 1;
			int y_inflated_min = (y_ - h_inflation) - 1;
			int z_inflated_max = (z_ + v_inflation) + 1;
			int z_inflated_min = (z_ - v_inflation) - 1;

			// Loop 'x axis' by 'x axis' for all discrete occupancy matrix cube around the node that must be inflated
			// Due to the inflated occupancy matrix size increment, the inside checking is not neccesary
			for(int j = y_inflated_min; j <= y_inflated_max; j++)
				for(int k = z_inflated_min; k <= z_inflated_max; k++)
				{
					unsigned int world_index_x_inflated_min = getWorldIndex(x_inflated_min, j, k);
					memset(&discrete_world[world_index_x_inflated_min], 0, (x_inflated_max - x_inflated_min)*sizeof(ThetaStartNodeLink3D)); 
				}
		}

		/**
		  Inflate a occupied cells filling all cells inside the around cylinder in 
		  the occupancy matrix. 
			@param discrete position of the cell to inflate
		**/
		inline void inflateNodeAsCylinder(int &x_, int &y_, int &z_)
		{
			// Get discretized radius of the inflation cylinder
			int R = h_inflation;
			
			// Inflation limits around the node
			int x_inflated_max = (x_ + h_inflation) + 1;
			int x_inflated_min = (x_ - h_inflation) - 1;
			int y_inflated_max = (y_ + h_inflation) + 1;
			int y_inflated_min = (y_ - h_inflation) - 1;
			int z_inflated_max = (z_ + v_inflation) + 1;
			int z_inflated_min = (z_ - v_inflation) - 1;
			
			// Loop throug inflation limits checking if it is inside the cylinder	
			// Due to the inflated occupancy matrix size increment, the inside checking is not neccesary
			for(int i = x_inflated_min; i <= x_inflated_max; i++)
				for(int j = y_inflated_min; j <= y_inflated_max; j++)
					for(int k = z_inflated_min; k <= z_inflated_max; k++)
					{
						if(isInsideTheCylinder(i,j, x_,y_, R))
						{
							unsigned int world_index_ = getWorldIndex(i, j, k);
							discrete_world[world_index_].notOccupied = false;
							discrete_world[world_index_].lastTimeSeen = 0;
						}
					}
		}

		/**
		  Inflate a occupied cells filling all cells around in the occupancy matrix ONLY HORIZONTALLY
			@param discrete position of the cell to inflate
		**/
		inline void inflateNodeAsXyRectangle(int &x_, int &y_, int &z_)
		{
			// Inflation limits
			int x_inflated_max = (x_ + h_inflation) + 1;
			int x_inflated_min = (x_ - h_inflation) - 1;
			int y_inflated_max = (y_ + h_inflation) + 1;
			int y_inflated_min = (y_ - h_inflation) - 1;

			// Due to the inflated occupancy matrix size increment, the inside checking is not neccesary
			for(int i = x_inflated_min; i <= x_inflated_max; i++)
				for(int j = y_inflated_min; j <= y_inflated_max; j++)
				{
					unsigned int world_index_ = getWorldIndex(i, j, z_);
					discrete_world[world_index_].notOccupied = false;
				}
		}

		/**
		  Check if the [x,y] position is inside the cylinder [xo,yo,R] 
		  (really is circle, yes...)
			@return true if it is inside
		**/
		inline bool isInsideTheCylinder(int &x, int &y, int &xo, int &yo, int &R)
		{
			int R_ = sqrt((x - xo)*(x - xo) + (y - yo)*(y - yo));
			
			if(R_ <= R)
				return true;
			else
				return false;
		}

		/**
		 Set and search a valid discrete initial/final position in a horizontal ring 
		 centered in '(xs,ys,zs)' and radius 'd'
		   @param [x,y,z] center
		   @param 'd' search radius
		   @return true if is a valid initial/final position and has been set correctly
		**/
		inline bool searchInitialPositionInXyRing(int xs, int ys, int zs, int d)
		{
			// 2 sides of the rectangular ring ..
			for (int i = 0; i < d + 1; i++)
			{
				DiscretePosition p1;
				p1.x = xs - d + i;
				p1.y = ys - i;
				p1.z = zs;
				
				if(setValidInitialPosition(p1))
					return true;
				
				if(d != 0)	// if level is zero, is the same that previous point check
				{
					p1.x = xs + d - i;
					p1.y = ys + i;

					if(setValidInitialPosition(p1))
						return true;
				}
			}
			
			// .. and the other ones
			for (int i = 1; i < d; i++)
			{
				DiscretePosition p2;
				p2.x = xs - i;
				p2.y = ys + d - i;
				p2.z = zs;

				if(setValidInitialPosition(p2))
					return true;

				p2.x = xs + d - i;
				p2.y = ys - i;
				
				if(setValidInitialPosition(p2))
					return true;
			}
			
			return false;
		}

		inline bool searchInitialPositionInXyRingBack(int xs, int ys, int zs, int d)
		{
			// 2 sides of the rectangular ring ..
			for (int i = 0; i < d + 1; i++)
			{
				DiscretePosition p1;
				p1.x = xs - d + i;
				p1.y = ys - i;
				p1.z = zs;
				
				if(p1.x < -1)
					if(setValidInitialPosition(p1))
						return true;
				
				if(d != 0)	// if level is zero, is the same that previous point check
				{
					p1.x = xs + d - i;
					p1.y = ys + i;
					
					if(p1.x < -1)	
						if(setValidInitialPosition(p1))
							return true;
				}
			}
			
			// .. and the other ones
			for (int i = 1; i < d; i++)
			{
				DiscretePosition p2;
				p2.x = xs - i;
				p2.y = ys + d - i;
				p2.z = zs;

				if(p2.x < -1)
					if(setValidInitialPosition(p2))
						return true;

				p2.x = xs + d - i;
				p2.y = ys - i;
				
				if(p2.x < -1)
					if(setValidInitialPosition(p2))
						return true;
			}
			
			return false;
		}

		inline bool searchFinalPositionInXyRing(int xs, int ys, int zs, int d)
		{
			// 2 sides of the rectangular ring ..
			for (int i = 0; i < d + 1; i++)
			{
				DiscretePosition p1;
				p1.x = xs - d + i;
				p1.y = ys - i;
				p1.z = zs;
				
				if(setValidFinalPosition(p1))
					return true;
				
				if(d != 0)	// if level is zero, is the same that previous point check
				{
					p1.x = xs + d - i;
					p1.y = ys + i;

					if(setValidFinalPosition(p1))
						return true;
				}
			}
			
			// .. and the other ones
			for (int i = 1; i < d; i++)
			{
				DiscretePosition p2;
				p2.x = xs - i;
				p2.y = ys + d - i;
				p2.z = zs;

				if(setValidFinalPosition(p2))
					return true;

				p2.x = xs + d - i;
				p2.y = ys - i;
				
				if(setValidFinalPosition(p2))
					return true;
			}
			
			return false;
		}

		inline bool searchFinalPositionInXyRingAhead(int xs, int ys, int zs, int d)
		{	
			// 2 sides of the rectangular ring ..
			for (int i = 0; i < d + 1; i++)
			{
				DiscretePosition p1;
				p1.x = xs - d + i;
				p1.y = ys - i;
				p1.z = zs;
				
				if(p1.x > 1)
					if(setValidFinalPosition(p1))
						return true;
				
				if(d != 0)	// if level is zero, is the same that previous point check
				{
					p1.x = xs + d - i;
					p1.y = ys + i;
					
					if(p1.x > 1)	
						if(setValidFinalPosition(p1))
							return true;
				}
			}
			
			// .. and the other ones
			for (int i = 1; i < d; i++)
			{
				DiscretePosition p2;
				p2.x = xs - i;
				p2.y = ys + d - i;
				p2.z = zs;

				if(p2.x > 1)
					if(setValidFinalPosition(p2))
						return true;

				p2.x = xs + d - i;
				p2.y = ys - i;
				
				if(p2.x > 1)
					if(setValidFinalPosition(p2))
						return true;
			}
			
			return false;
		}

		inline bool searchFinalPositionInXyRingAheadHorPriority(int xs, int ys, int zs, int d)
		{
			// Force search more horizontally
			return searchFinalPositionInXyRingAhead(xs, ys, zs, 2*d);
		}


  /** Variables **/	
  
		// Global Occupancy Matrix
		std::vector<ThetaStartNodeLink3D> discrete_world;	// Occupancy Matrix and its size 
		int matrix_size;
		int ws_x_max, ws_y_max, ws_z_max; // WorkSpace lenghts from origin (0,0,0)
		int ws_x_min, ws_y_min, ws_z_min;
		int h_inflation; // Inflation (Real and Safe distances from the MAV CoG)
		int v_inflation;
		int ws_x_max_inflated, ws_y_max_inflated, ws_z_max_inflated; // Inflated WorkSpace, the real size of the Occupancy Matrix
		int ws_x_min_inflated, ws_y_min_inflated, ws_z_min_inflated; // ... less isInside() checking
		int Lx, Ly, Lz;	// Inflated WorkSpace lenghts and theirs pre-computed inverses
		float Lx_inv, Ly_inv, Lz_inv;
		float step;	// Resolution of the Matrix and its inverse
		float step_inv;

		// Input octomap octree used to create the Occupancy Matrix
		//std::unique_ptr<octomap::AbstractOcTree> map;
		octomap::OcTree *map;
		// Origin and target position.
		Vector3 initial_position, final_position;	// Continuous
		ThetaStarNode3D *disc_initial, *disc_final;	// Discretes

		// Max time to get path
		int timeout;

		// Result path
		vector<Vector3> last_path;

		// Lazy Theta* with Optimization:  
		float goal_weight; // Reduction of the initial position distance weight C(s) = factor * g(s) + h(s) 
		float z_weight_cost; // Weight for height changes
		float z_not_inflate; // Altitude to not be inflated

		// Debug Visualization markers and theirs topics
		std::shared_ptr<ros::NodeHandle> nh; // Pointer to the process NodeHandle to publish topics
		RVizMarker marker; // Explored nodes by ThetaStar
		ros::Publisher marker_pub_;
		PointCloud occupancy_marker; // Occupancy Map as PointCloud markers
		ros::Publisher occupancy_marker_pub_;
		RVizMarker marker_no_los; // Explored nodes with no lineOfSight 
		ros::Publisher no_los_marker_pub_;

		// Trajectory parameters
		float dxy_max; // Maximum increment between wps [meters]
		float dz_max;
		float dxyz_tolerance; // Position tolerance for the path to trajectory segmentation [meters]
		float vm_xy; // Mean linear velocities [m/s]
		float vm_z;
		float vm_xy_1; // Mean linear velocities for initial and final displacement [m/s]
		float vm_z_1;
		float w_yaw; // Mean angular velocity at yaw [rad/s]
		float min_yaw_ahead; // Minimum position increment to set yaw ahead [meters]
		bool trajectoryParamsConfigured; // Flag to enable the Trajectory Computation

		// Flag to print the ROS_WARN()
		bool PRINT_WARNINGS;
};

} /* namespace PathPlanners */
#endif /* THETASTAR_H_ */
