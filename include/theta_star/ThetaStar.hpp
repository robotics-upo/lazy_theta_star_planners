/*
 * Copyright 2016 Ricardo Ragel de la Torre, GRVC, Univ. of Seville, Spain
 *
 * Resume: 	ThetaStar Class declarations and other auxiliar classes.
 * 			Motion Path Planner based on the Theta Star algoritm: 
 * 			"Weighted Lazy Theta Star with Optimization" and different
 * 			methods to get a colission-free path [x,y,z] or 
 * 			trajectory [xyz, yaw, time].
 */

#ifndef THETASTAR_H_
#define THETASTAR_H_

#include <vector>
#include <set>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>//Sustituir por un mensaje tipo vector2? 

#include <visualization_msgs/Marker.h>

#include <nav_msgs/OccupancyGrid.h>


#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


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
typedef trajectory_msgs::MultiDOFJointTrajectory 		Trajectory;
typedef trajectory_msgs::MultiDOFJointTrajectoryPtr 	TrajectoryPtr;
typedef trajectory_msgs::MultiDOFJointTrajectoryPoint 	TrajectoryPoint;
typedef visualization_msgs::Marker RVizMarker;


//*****************************************************************
//				Auxiliar Class for ThetaStar Algorithm
//*****************************************************************
class DiscretePosition{
	public:
	   int x,y;
};

// Nodes (declaration)
class ThetaStarNode;


class ThetaStartNodeLink
{
	public:
	   ThetaStartNodeLink(): 	node(NULL), isInOpenList(false), isInCandidateList(false), checked(false), 
								notOccupied(true), lastTimeSeen(std::numeric_limits<int>::max()), countToBeOccupied(0)
	   {
	   }

	   ThetaStarNode *node;				// Node from this link
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
class ThetaStarNode
{
	public:
	   ThetaStarNode():	parentNode(NULL), nodeInWorld(NULL), lineDistanceToFinalPoint( std::numeric_limits<float>::max() ), 
						distanceFromInitialPoint( std::numeric_limits<float>::max()), totalDistance(std::numeric_limits<float>::max())
	   {
	   }

	   DiscretePosition    point;		// Discrete position of this node
	   ThetaStarNode      *parentNode;	// pointer to parent node
	   ThetaStartNodeLink *nodeInWorld;	// pointer to link from this node to its parent 
	   float lineDistanceToFinalPoint;	// algorithm value
	   float distanceFromInitialPoint;	// algorithm value
	   float totalDistance;				// algorithm value
	   float cost; 						//Cost from costmap
	   // Comparator '!=' definition
	   friend bool operator != (const ThetaStarNode& lhs, const ThetaStarNode& rhs)
	   {
		  return lhs.point.x!=rhs.point.x ||
				 lhs.point.y!=rhs.point.y; 
				 
	   }
};

// Comparator for pointer to ThetaStarNode objects
struct NodePointerComparator
{
   bool operator () (const ThetaStarNode* const& lhs__, const ThetaStarNode* const& rhs__) const
   {
      const ThetaStarNode *lhs = lhs__;
      const ThetaStarNode *rhs = rhs__;
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
         res = lhs->parentNode->point.x - rhs->parentNode->point.x;
      }
      if(res==0)
      {
         res = lhs->parentNode->point.y - rhs->parentNode->point.y;
      }
     
      return res < 0;
   }
};


//*****************************************************************
// 				ThetaStar Algoritm Class Declaration
//*****************************************************************
class ThetaStar
{
  public:
		/**
		  Default constructor
		**/	
		ThetaStar();

		/**
		  Constructor with arguments
		   @param planner name for topic names 
		   @param frame_id for debug markers 
		   @param simetric or asimetric workspace centered at (0,0,0) [meters]
		   @param occupancy matrix resolution [meters]
		   @param occupancy matrix nodes inflation (horizontal and vertical, real + safety) [meters]
		   @param Lazy Theta* with Optimization: goal point factor [0 to inf]. Bigger -> distance to target more weight than distance to origin -> minor exploration -> shorter runtime, grater path length
		   @param NodeHandle 
		**/
		ThetaStar(char* plannerName, char* frame_id, 
		float ws_x_max_, float ws_y_max_, float ws_x_min_, float ws_y_min_, 
		float step_, float goal_weight_, ros::NodeHandle *n);

		/**
		  Initialization
		   @param planner name for topic names 
		   @param frame_id for debug markers 
		   @param simetric or asimetric workspace centered at (0,0,0) [meters]
		   @param occupancy matrix resolution [meters]
		   @param occupancy matrix nodes inflation (horizontal and vertical, real + safety) [meters]
		   @param Lazy Theta* with Optimization: goal point factor [0 to inf]. Bigger -> distance to target more weight than distance to origin -> minor exploration -> shorter runtime, grater path length
		   @param NodeHandle 
		**/ 
		void init(char* plannerName, char* frame_id, 
		float ws_x_max_, float ws_y_max_, float ws_x_min_, float ws_y_min_, 
		float step_, float goal_weight_, ros::NodeHandle *n);

		/**
		  Default destructor
		**/
		~ThetaStar();

		/**
		 Configure the computed trajectories by getCurrentTrajectory() functions
			@param dxy_max, dz_max: Maximum increment between wps [meters]
			@param dxy_tolerance: 	position tolerance for the path to trajectory segmentation [meters]
			@param vm_xy: 			Mean linear velocities [m/s]
			@param vm_xy_1:			Mean linear velocities for initial and final displacement [m/s]
			@param w_yaw:			Mean angular velocity at yaw [rad/s]
			@param min_yaw_ahead:	Minimum position increment to set yaw ahead [meters]
		**/
		void setTrajectoryParams(float dxy_max_, float dxy_tolerance_, float min_yaw_ahead_);
		/**
		 Read map and set up discrete world with occupancy matrix from map_server
		 	@param message:			Occupancy grid message from map_server
		 **/
		void getMap(nav_msgs::OccupancyGrid message);

		/** 
		   Clear occupancy discrete matrix
		**/
		void clearMap();

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
			if(setInitialPosition(p)){
				if(!isInitialPositionOccupied()){
					//ROS_INFO("ThetaStar: Initial discrete position [%d, %d] set correctly", p.x,p.y); 
					return true;
				}
			}else{
				if(PRINT_WARNINGS)
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
			if(setFinalPosition(p)){
				if(!isFinalPositionOccupied()){
					//ROS_INFO("ThetaStar: Final discrete position [%d, %d] set correctly", p.x,p.y);
					return true;
				}
			}else{
				if(PRINT_WARNINGS)
					ROS_WARN("ThetaStar: Final position outside the workspace attempt!!");
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
		bool searchInitialPosition2d(float maxDistance); 		// Symetric search in nearest XY neighbours //Puede que nos podamos quedar con esta funcion, adapatandola y eliminar la 3d y 3dback directamente
		

		/**
		 Check if the current final position is VALID and, if it
		 is not valid, search and set a free initial position
		 around it.
		   @param maximum distance from initial position
		   @return true if a initial valid position has been set
		**/
		bool searchFinalPosition2d(float maxDistance); 						// Symetric search in nearest XY neighbours //Lo mismo que en el caso de initial position
		

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
		vector<Vector3> getCurrentPath();//Cambiar los elementos del contenedor por los correspondientes
	
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
		void getNeighbors(ThetaStarNode &node, set<ThetaStarNode*,NodePointerComparator> &neighbors);

		/**
		 Returns distance to goal.
		   @param node to calculate distance from.
		   @return distance to goal.
		 Weighted version, using the 'z_weight_cost' param to increase the altitude change cost
		**/
		float distanceToGoal(ThetaStarNode node);
		float weightedDistanceToGoal(ThetaStarNode node);

		/**
		 Returns distance between 2 nodes.
		   @param origin node
		   @param target node
		   @return distance to origin to target node
		 Weighted version, using the 'z_weight_cost' param to increase the altitude change cost
		**/
		float distanceBetween2nodes(ThetaStarNode &n1,ThetaStarNode &n2);
		float weightedDistanceBetween2nodes(ThetaStarNode &n1,ThetaStarNode &n2); 

		/**
		 Returns distance from initial.
		   @param node
		   @param his parent
		   @return distance to init node
		 Weighted version, using the 'z_weight_cost' param to increase the altitude change cost
		**/
		float distanceFromInitialPoint(ThetaStarNode node, ThetaStarNode parent);
		float weightedDistanceFromInitialPoint(ThetaStarNode node, ThetaStarNode parent);

		/**
		 Open and candidates node list, ordered by minor total distance (std::set --> listas ordenadas)
		**/
		set<ThetaStarNode*, NodePointerComparator> open,candidates;

		/**
		 Main algorithm functions. See http://aigamedev.com/open/tutorial/lazy-theta-star/
		**/
		void ComputeCost(ThetaStarNode &s, ThetaStarNode &s2);
		void UpdateVertex(ThetaStarNode &s, ThetaStarNode &s2);
		void SetVertex(ThetaStarNode &s,set<ThetaStarNode*,NodePointerComparator> &neighbors);
		double g(ThetaStarNode &s);

		/**	
		  Returns if exists line of sight between two points.
			@param Position of first one point
			@param Position of second one point
			@return true if exists line of sight, false in the other case.
		**/
		bool lineofsight(ThetaStarNode &p1, ThetaStarNode &p2);

		/** 
		 Check if a node is occupied
		   @param A ThetaStarNode
		   @return true if is occupied in the occupancy matrix
		**/
		bool isOccupied(ThetaStarNode n);

		/** 
		  Return true if set initial/final position is occupied
			@return true if set initial/final position is occupied
		**/
		bool isInitialPositionOccupied();
		bool isFinalPositionOccupied();

		//! Aux Function: Tajectory point to fill and send to the trajectory tracker sight ahead -- Yaw is calculated in such a way that cameras always see ahead
		void setPositionYawAndTime(TrajectoryPoint &trajectory_point, Vector3 position, double _yaw);

		//! Aux Function: return the horizonatl module of [X,Y]
		double getHorizontalNorm(double x, double y);

		//! Aux Function: return the horizontal distance from Point1 to Point2
		double getHorizontalDistance(TrajectoryPoint P1, TrajectoryPoint P2);

		

		//! Aux Function: Get differential from last_yaw to next_yaw in [-PI, PI], but in absolute value --> [0, PI] 
		double getDyaw(double next_yaw, double last_yaw);

		//! Aux Function: simply print the trajectory msg data
		void printfTrajectory(Trajectory trajectory, string trajectory_name);

		/**
		 Publish a marker in the position of a ThetaStarNode
		   @param node to publish his position
		   @param Set true to publish it instantly or false to simply push back at the marker array to publish later
		**/
		void publishMarker(ThetaStarNode &s, bool publish);


  /** Inline Functions **/

		/**
		 Get discrete occupancy matrix index for this discrete (x,y,z) position
		   @param x, y, z discrete position values
		   @return the index in the occupancy matrix
		**/
		inline unsigned int getWorldIndex(int &x, int &y) //ELiminar la z
		{
			//return (unsigned int)((x - ws_x_min_inflated) + (Lx)*((y - ws_y_min_inflated))); 
			return (unsigned int)(Lx*y +x );
		}

		/**
		 Get discrete (x,y,z) position for a discrete occupancy matrix index
		   @param x, y, z discrete position values (output)
		   @param index (input)
		**/
		inline void getDiscreteWorldPositionFromIndex(int &x, int &y, int index)
		{
			// Discrete matrix = [(ALL_X_SEGMENT)(ALL_X_SEGMENT)...(ALL_X_SEGMENT) | (ALL_X_SEGMENT)(ALL_X_SEGMENT)...(ALL_X_SEGMENT)| ............. |(ALL_X_SEGMENT)(ALL_X_SEGMENT)...(ALL_X_SEGMENT)]
			//		for			   Y=WS_Y_MIN	  Y=WS_Y_MIN+1  ... Y=WS_Y_MAX     |  Y=WS_Y_MIN	  Y=WS_Y_MIN+1  ... Y=WS_Y_MAX   | ............. | Y=WS_Y_MIN	  Y=WS_Y_MIN+1  ... Y=WS_Y_MAX    
			//		for			   				  Z = WS_Z_MIN 					   |                  Z = WS_Z_MIN+1				 | ............. |            Z = WS_Z_MAX = WS_Z_MIN+Lz      
			
			//Review index calculation and matrix index calculation					
			//int z_n_segmts = floor( index * Lx_inv * Ly_inv );					// Index z segment (0 to Lz)
			//int y_n_segmts = floor( (index - z_n_segmts * Lx * Ly) * Ly_inv );	// Index y segment (0 to Ly)
			
			//x = ws_x_min_inflated + (index - z_n_segmts*Lx*Ly - y_n_segmts*Lx);
			//y = ws_y_min_inflated + y_n_segmts;
			//z = ws_z_min_inflated + z_n_segmts; // quitar?
			//Provisional para map_server map con tamaï¿½o fijo conocido
<<<<<<< HEAD
			int y_n_segmts = floor( index/Lx-3 );	// Index y segment (0 to Ly)
=======
			int y_n_segmts = floor( index  /(Lx-3) );	// Index y segment (0 to Ly)
>>>>>>> fc18f9421c42ae8d5daab0253c4a1fab3f63ce97
			
			x =  (index -  y_n_segmts*Lx);
			y =  y_n_segmts;
		}

		/**
		 Check if a Node/continuousPosition/discretePosition is inside the worksspace
		   @param ThetaStar Node / Discrete position
		   @return true if is inside the ws
		**/
		inline bool isInside(ThetaStarNode n)
		{
			return  isInside(n.point.x,n.point.y); //Qutar z
		}
		inline bool isInside(Vector3 &v) //Sustituir vector3
		{
			int x_ = v.x*step_inv;
			int y_ = v.y*step_inv;
			
			return  isInside(x_, y_); //Quitar z
		}
		inline bool isInside(int &x, int &y) //quitar z
		{
			return  (x < (ws_x_max-1) && x > (ws_x_min+1)) &&
					(y < (ws_y_max-1) && y > (ws_y_min+1)); 
		}
		/**
		  Check if the [x,y] position is inside the circle [xo,yo,R] 
			@return true if it is inside
		**/
		inline bool isInsideTheCircle(int &x, int &y, int &xo, int &yo, int &R)
		{
			int R_ = sqrt((x - xo)*(x - xo) + (y - yo)*(y - yo));
			
			if(R_ <= R)
				return true;
			else
				return false;
		}
		/**
		 Set and search a valid discrete initial/final position in a horizontal ring 
		 centered in '(xs,ys)' and radius 'd'
		   @param [x,y] center
		   @param 'd' search radius
		   @return true if is a valid initial/final position and has been set correctly
		**/
		inline bool searchInitialPositionInXyRing(int xs, int ys, int d)
		{
			// 2 sides of the rectangular ring ..
			for (int i = 0; i < d + 1; i++)
			{
				DiscretePosition p1;
				p1.x = xs - d + i;
				p1.y = ys - i;
				
				
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

				if(setValidInitialPosition(p2))
					return true;

				p2.x = xs + d - i;
				p2.y = ys - i;
				
				if(setValidInitialPosition(p2))
					return true;
			}
			
			return false;
		}

		inline bool searchInitialPositionInXyRingBack(int xs, int ys, int d)
		{
			// 2 sides of the rectangular ring ..
			for (int i = 0; i < d + 1; i++)
			{
				DiscretePosition p1;
				p1.x = xs - d + i;
				p1.y = ys - i;
		
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

		inline bool searchFinalPositionInXyRing(int xs, int ys, int d)
		{
			// 2 sides of the rectangular ring ..
			for (int i = 0; i < d + 1; i++)
			{
				DiscretePosition p1;
				p1.x = xs - d + i;
				p1.y = ys - i;
				
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

				if(setValidFinalPosition(p2))
					return true;

				p2.x = xs + d - i;
				p2.y = ys - i;
				
				if(setValidFinalPosition(p2))
					return true;
			}
			
			return false;
		}

		inline bool searchFinalPositionInXyRingAhead(int xs, int ys, int d)
		{	
			// 2 sides of the rectangular ring ..
			for (int i = 0; i < d + 1; i++)
			{
				DiscretePosition p1;
				p1.x = xs - d + i;
				p1.y = ys - i;
				
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

		inline bool searchFinalPositionInXyRingAheadHorPriority(int xs, int ys, int d)
		{
			// Force search more horizontally
			return searchFinalPositionInXyRingAhead(xs, ys, 2*d);
		}


  /** Variables **/	
  
		// Global Occupancy Matrix
		std::vector<ThetaStartNodeLink> discrete_world;	// Occupancy Matrix and its size 
		int matrix_size;
		int ws_x_max, ws_y_max; // WorkSpace lenghts from origin (0,0,0)
		int ws_x_min, ws_y_min;
		
		int Lx, Ly;	// Inflated WorkSpace lenghts and theirs pre-computed inverses
		float Lx_inv, Ly_inv;
		float step;	// Resolution of the Matrix and its inverse
		float step_inv;

		// Input octomap octree used to create the Occupancy Matrix
		
		// Origin and target position.
		Vector3 initial_position, final_position;	// Continuous
		ThetaStarNode *disc_initial, *disc_final;	// Discretes

		// Max time to get path
		int timeout;

		// Result path
		vector<Vector3> last_path;

		// Lazy Theta* with Optimization:  
		float goal_weight; // Reduction of the initial position distance weight C(s) = factor * g(s) + h(s) 
		

		// Debug Visualization markers and theirs topics
		ros::NodeHandle *nh; // Pointer to the process NodeHandle to publish topics
		RVizMarker marker; // Explored nodes by ThetaStar
		ros::Publisher marker_pub_;
		RVizMarker marker_no_los; // Explored nodes with no lineOfSight 
		ros::Publisher no_los_marker_pub_;

		// Trajectory parameters
		float dxy_max; // Maximum increment between wps [meters]
		float dz_max;
		float dxy_tolerance; // Position tolerance for the path to trajectory segmentation [meters]
	
		float min_yaw_ahead; // Minimum position increment to set yaw ahead [meters]
		float path_length;
		bool trajectoryParamsConfigured; // Flag to enable the Trajectory Computation

		// Flag to print the ROS_WARN()
		bool PRINT_WARNINGS;
};

} /* namespace PathPlanners */
#endif /* THETASTAR_H_ */
