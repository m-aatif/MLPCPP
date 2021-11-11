#pragma once

#include <stdint.h>

#include "Planner.h"
#include "Geometry.h"

using namespace std;
using namespace geon;

#define	DEFAULT_ASTAR_HEURISTIC_WEIGHT	(0.5)

namespace afarcloud
{
	/**
	AStarNode: A class that contains the node parameters used by A* planner algorithm.
	*/
	class AStarNode
	{
	public:
		AStarNode();
		AStarNode(aPoint ptStart, double cost, int32_t parentIndex);

		inline aPoint origin() const { return m_ptStart; }
		inline double cost() const { return m_cost; }
		inline int32_t parentIndex() const { return m_parentIndex; }
		inline double x() const { return m_ptStart.x(); }
		inline double y() const { return m_ptStart.y(); }
		
		inline void setCost(double cost) { m_cost = cost; }
		inline void setParentIndex(int32_t index) { m_parentIndex = index; }

	private:
		aPoint m_ptStart;
		double m_cost;
		int32_t m_parentIndex;
	};

	/**
	AStarMotion: A class that contains the elementary motion step parameters used by A* planner algorithm.
	*/
	class AStarMotion
	{
	public:
		AStarMotion(double x, double y, double cost, double scaling);

		inline double x() const { return m_x; }
		inline double y() const { return m_y; }
		inline double cost() const { return m_cost; }
	private:
		double m_x;
		double m_y;
		double m_cost;
	};

	typedef map<int32_t, AStarNode> nodeMap;
	typedef std::vector<AStarNode> nodeVec;
	typedef std::vector<AStarMotion> motionVec;

	/**
	AStarPlanner: A class that implements the A* planner algorithm.
	*/
	class AStarPlanner : public Planner
	{
	public:
		AStarPlanner(bool bAutoScale = false) { m_bAutoScale = bAutoScale; m_bInitialized = false; }
		AStarPlanner(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, double resolution, double heuristicWeight = 1.0, bool bAutoScale = false);
		pointVec plan();
		pointVec plan(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, double resolution, double heuristicWeight = 1.0);
		static int m_logNumber;

	private:
		void init(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, double resolution, double heuristicWeight);
		int32_t calcGridIndex(AStarNode node);
		bool collides(AStarNode baseNode, AStarNode nearNode);
		bool collides(aPoint p1, aPoint p2);
		double calcHeuristic(AStarNode node1, AStarNode node2);
		int32_t getOpenSetLowestCostIndex(AStarNode goalNode);
		bool verifyNode(AStarNode node, AStarNode current);
		pointVec calcFinalPath(AStarNode goalNode);
		pointVec prunePath(pointVec path);
		void setMotion();

		bool m_bAutoScale;				///	Not used
		bool m_bInitialized;			///	Initialization flag.

		AStarNode m_startNode;			///	The start node.
		AStarNode m_goalNode;			///	The goal node.
		rectangleVec m_obstacles;		///	The vector of obstacles.
		motionVec m_motion;				///	The vector of possible motions.
		nodeMap m_openSet;				///	The A* Nodes Open Set.
		nodeMap m_closedSet;			///	The A* Nodes Closed Set.
		aRectangle m_rectField;			///	The field of application.
		double m_resolution;			///	The resolution.
		double m_heuristicWeight;		///	The heuristic weight.

		double m_min_x;
		double m_min_y;
		double m_max_x;
		double m_max_y;
		double m_width;
		double m_height;

	};

}