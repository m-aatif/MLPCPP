#include "AStarPlanner.h"

#include <boost\math\special_functions\hypot.hpp>

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <fstream>

#include "Configurator.h"

namespace afarcloud
{
	/*****************************************************************************************
	* AStarNode::AStarNode()
	* Summary: A* Node default constructor. It represents a node with origin=(0,0), cost=0 and parent index=-1.
	* Params: none.
	* Return: none
	*****************************************************************************************/
	AStarNode::AStarNode()
	{
		m_ptStart = aPoint(0, 0);
		m_cost = 0;
		m_parentIndex = -1;
	}

	/**
	AStarNode::AStarNode(aPoint ptStart, double cost, int32_t parentIndex)
	Summary: A* Node specialized constructor. It represents a node with origin=<ptStart>, cost=<cost>
	and parent index=<parentIndex>.
	Params: ptStart		aPoint		The node origin.
			cost		double		The cost of the node.
			parentIndex	int32_t		Node's parent index.
	Return: none
	*/
	AStarNode::AStarNode(aPoint ptStart, double cost, int32_t parentIndex)
	{
		m_ptStart = ptStart;
		m_cost = cost;
		m_parentIndex = parentIndex;
	}

	/**
	AStarMotion::AStarMotion(double x, double y, double cost, double scaling)
	Summary: A* Motion constructor. It represents an elementary motion step by <x> and <y> with cost=<cost> scaled
	by <scaling>.
	Params: x			double		The elementary motion in X direction.
			y			double		The elementary motion in Y direction
			cost		double		The cost of the step.
			scaling		double		The scaling of cost.
	Return: none
	*/
	AStarMotion::AStarMotion(double x, double y, double cost, double scaling)
	{
		m_x = x;
		m_y = y;
		m_cost = cost * scaling;
	}

	int AStarPlanner::m_logNumber = 0;

	/**
	AStarPlanner::AStarPlanner(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, double resolution, double heuristicWeight, bool bAutoScale)
	Summary: A* Planner constructor.
	Params: ptStart			aPoint			See Planner constructor.
			ptGoal			aPoint			See Planner constructor.
			obstacles		rectangleVec	See Planner constructor.
			rectField		aRectangle		See Planner constructor.
			resolution		double			Algorithm's resolution.
			heuristicWeight	double			Algorithm's heuristic weight.
	Return: none
	*/
	AStarPlanner::AStarPlanner(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, double resolution, double heuristicWeight, bool bAutoScale)
	{
		m_bAutoScale = bAutoScale;
		init(ptStart, ptGoal, obstacles, rectField, resolution, heuristicWeight);
	}

	/**
	void AStarPlanner::init(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, double resolution, double heuristicWeight)	Summary: A* Planner constructor.
	Summary: Initializes the variables used by A* Planner algorithm.
	Params: ptStart			aPoint			See Planner constructor.
			ptGoal			aPoint			See Planner constructor.
			obstacles		rectangleVec	See Planner constructor.
			rectField		aRectangle		See Planner constructor.
			resolution		double			See AStarPlanner constructor.
			heuristicWeight	double			AStarPlanner constructor.
	Return: none
	*/
	void AStarPlanner::init(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, double resolution, double heuristicWeight)
	{
		double dist, scalingFactor;

		scalingFactor = 1.0;

		/**
		Auto scaling not used !
		*/
		if (m_bAutoScale)
		{
			dist = hypot(rectField.width(), rectField.height());
			while (dist < 100)
			{
				scalingFactor *= 10.0;
				rectField.scaleUp(10.0);
				dist = hypot(rectField.width(), rectField.height());
			}
			m_resolution = dist / resolution;
		}
		else
			m_resolution = resolution;

		/**
		Scaling of all variables by m_resolution.
		*/
		m_min_x = rectField.left();
		m_min_y = rectField.bottom();

		m_rectField = GeometryTools::scaleRect(rectField, m_min_x, m_min_y, m_resolution);

		m_max_x = rectField.right();
		m_max_y = rectField.top();
		m_width = (m_max_x - m_min_x) / m_resolution;
		m_height = (m_max_y - m_min_y) / m_resolution;
		m_max_x = m_min_x + m_width;
		m_max_y = m_min_y + m_height;

		aRectangle rcx;
		rectangleVec::iterator ito = obstacles.begin();
		while (ito != obstacles.end())
		{
			rcx = GeometryTools::scaleRect(*ito, m_min_x, m_min_y, m_resolution);
			m_obstacles.push_back(rcx);
			ito++;
		}

		m_startNode = AStarNode(GeometryTools::scalePoint(ptStart, m_min_x, m_min_y, m_resolution), 0.0, -1);
		m_goalNode = AStarNode(GeometryTools::scalePoint(ptGoal, m_min_x, m_min_y, m_resolution), 0.0, -1);

		m_heuristicWeight = heuristicWeight;
		setMotion();
		m_bInitialized = true;
	}

	/**
	* double AStarPlanner::calcHeuristic(AStarNode node1, AStarNode node2)
	* Summary: Calculates the heuristic weight for <node1> -> <node2> motion.
	* Params: node1			AStarNode		The first node.
	* 		  node2			AStarNode		The second node.
	* Return:				double			The heuristic cost for that motion.
	*/
	double AStarPlanner::calcHeuristic(AStarNode node1, AStarNode node2)
	{
		return m_heuristicWeight * hypot(node1.x() - node2.x(), node1.y() - node2.y());
	}

	/**
	* int32_t AStarPlanner::getOpenSetLowestCostIndex(AStarNode goalNode)
	* Summary: Gets the index in field grid of the node, contained in open set, whose cost is minimal in distance 
	* from <goalNode>.
	* Params: goalNode		AStarNode		The goal node.
	* 		 node2			AStarNode		The second node.
	* Return:				int32_t			The node index which satisfied the minimum cost condition.
	*/
	int32_t AStarPlanner::getOpenSetLowestCostIndex(AStarNode goalNode)
	{
		int32_t retVal = -1;
		double minCost = 1.0E10, cost;
		nodeMap::iterator it = m_openSet.begin();
		while (it != m_openSet.end())
		{
			cost = (it->second).cost() + calcHeuristic(goalNode, it->second);
			if (cost < minCost)
			{
				minCost = cost;
				retVal = it->first;
			}
			it++;
		}

		return retVal;
	}

	/**
	* bool AStarPlanner::verifyNode(AStarNode node, AStarNode current)
	* Summary: Verifies if a <node> -> <current> motion is inside the field area an if it collides with an obstacle.
	* Params: node			AStarNode		The node to check.
	*		  current		AStarNode		The current node.
	* Return:				bool			TRUE if the motion is allowed.
	*/
	bool AStarPlanner::verifyNode(AStarNode node, AStarNode current)
	{
		if (node.x() < m_rectField.left())
			return false;
		if (node.y() < m_rectField.bottom())
			return false;
		if (node.x() > m_rectField.right())
			return false;
		if (node.y() > m_rectField.top())
			return false;

		if (collides(node, current))
			return false;

		return true;
	}

	/**
	* int32_t AStarPlanner::calcGridIndex(AStarNode node)
	* Summary: Calculate the index of the <node> in the field area grid.
	* Params: node			AStarNode		The node whose index has to be calculated.
	* Return:				int32_t			The index in grid.
	*/
	int32_t AStarPlanner::calcGridIndex(AStarNode node)
	{
		return (int32_t)(node.y() * m_rectField.width() + node.x());
	}

	/**
	bool AStarPlanner::collides(AStarNode baseNode, AStarNode nearNode)
	Summary: Verifies if a <baseNde> -> <nearNode> motion intersects an obstacle.
	Params: baseNode		AStarNode		The node to check.
			nearNode		AStarNode		The near node.
	Return:					bool			TRUE if the motion is allowed.
	*/
	bool AStarPlanner::collides(AStarNode baseNode, AStarNode nearNode)
	{
		aRectangle rc;
		rectangleVec::iterator ito = m_obstacles.begin();
		while (ito != m_obstacles.end())
		{
			rc = *ito;
			if (rc.intersect2(baseNode.origin(), nearNode.origin()))
				return true;
			ito++;
		}
		return false;
	}

	/**
	bool AStarPlanner::collides(aPoint p1, aPoint p2)
	Summary: Verifies if a <p1> -> <p2> motion intersects an obstacle.
	Params: p1				aPoint			The point to check.
			p2				aPoint			The near point.
	Return:					bool			TRUE if the motion is allowed.
	*/
	bool AStarPlanner::collides(aPoint p1, aPoint p2)
	{
		aRectangle rc;
		rectangleVec::iterator ito = m_obstacles.begin();
		while (ito != m_obstacles.end())
		{
			rc = *ito;
			if (rc.intersect2(p1, p2))
				return true;
			ito++;
		}
		return false;
	}

	/**
	pointVec AStarPlanner::calcFinalPath(AStarNode goalNode)
	Summary: Reconstructs the path contained in closed set, staating from <goalNode> and tracking back to the node
	with no parent. Each found node point is de-scaled by m_resolution.
	Params: goalNode		AStarNode		The node to start the search.
	Return:					pointVec		The reconstructed path.
	*/
	pointVec AStarPlanner::calcFinalPath(AStarNode goalNode)
	{
		pointVec path;
		path.push_back(GeometryTools::descalePoint(goalNode.origin(), m_min_x, m_min_y, m_resolution));
		int32_t parentIndex = goalNode.parentIndex();
		while (parentIndex != -1)
		{
			AStarNode n = m_closedSet[parentIndex];
			path.push_back(GeometryTools::descalePoint(n.origin(), m_min_x, m_min_y, m_resolution));
			parentIndex = n.parentIndex();
		}
		return path;
	}

	/**
	pointVec AStarPlanner::prunePath(pointVec path)
	Summary: Prunes the <path> verifying if some points collide with obstacles.
	Then prunes the obtained path, deleting the points belonging to the same line.
	Finally, reverses the path.
	Params: path			pointVec		The path to prune.
	Return:					pointVec		The pruned and ordered path.
	*/
	pointVec AStarPlanner::prunePath(pointVec path)
	{
		pointVec ppath;
		aPoint p1, p2;
		pointVec::iterator it = path.begin();
		bool bAdded = true;

		/**
		Checks if the path consists of only two points.
		*/
		if (path.size() <= 2)
		{
			std::reverse(path.begin(), path.end());
			return path;
		}

		/**
		First pruning.
		*/
		p1 = *it;
		ppath.push_back(p1);
		it++;
		while (it != path.end())
		{
			p2 = *it;
			it++;
			bAdded = false;
			if (!collides(p1, p2))
			{
				ppath.push_back(p2);
				p1 = p2;
				bAdded = true;
			}
		}
		if (!bAdded)
			ppath.push_back(p2);

		/**
		Second pruning.
		*/
		aPoint pt0, pt1, pt2;
		bool bBelongs;
		pointVec outPath;

		pt0 = ppath[0];
		outPath.push_back(pt0);
		pt1 = ppath[1];
		for (size_t n = 2; n < ppath.size(); n++)
		{
			pt2 = ppath[n];
			bBelongs = GeometryTools::belongsToLine(pt0, pt1, pt2);
			if (bBelongs)
			{
				pt1 = pt2;
			}
			else
			{
				outPath.push_back(pt1);
				pt0 = pt1;
				pt1 = pt2;
			}
		}
		outPath.push_back(pt2);
		/**
		Path reversal.
		*/
		std::reverse(outPath.begin(), outPath.end());

		return outPath;
	}

	/**
	void AStarPlanner::setMotion()
	Summary: Sets up the motion vector. It fills the <m_motion> vector with all possible moves.
	Params: none.
	Return:	none.
	*/
	void AStarPlanner::setMotion()
	{
		m_motion.push_back(AStarMotion(1, 0, 1, m_resolution));				///	E  motion
		m_motion.push_back(AStarMotion(0, 1, 1, m_resolution));				///	N  motion
		m_motion.push_back(AStarMotion(-1, 0, 1, m_resolution));			/// W	motion
		m_motion.push_back(AStarMotion(0, -1, 1, m_resolution));			/// S  motion
		m_motion.push_back(AStarMotion(-1, -1, sqrt(2), m_resolution));		/// SW motion
		m_motion.push_back(AStarMotion(-1, 1, sqrt(2), m_resolution));		///	NW motion
		m_motion.push_back(AStarMotion(1, -1, sqrt(2), m_resolution));		/// SE motion
		m_motion.push_back(AStarMotion(1, 1, sqrt(2), m_resolution));		///	NE motion
	}

	/**
	pointVec AStarPlanner::plan()
	Summary: The planner algorithm.
	Params: none.
	Return:	path		pointVec		The path obtained.
	*/
	pointVec AStarPlanner::plan()
	{
		pointVec path;

		/**
		Initialization check.
		*/
		if (!m_bInitialized)
			return path;

		AStarNode node, currentNode;
		AStarNode startNode = m_startNode;
		AStarNode goalNode = m_goalNode;

		int32_t c_id;
		std::ofstream _logfile;

		if (m_bLog)
		{
			std::ostringstream oss;
			oss << Configurator::getLogFolder() << "\\ASTAR" << m_logNumber << ".log";
			_logfile = std::ofstream(oss.str());
		}
		
		/**
		Verifies if the path <startNode> -> <goalNode> is clear.
		*/
		if (!collides(startNode, goalNode))
		{
			path.push_back(GeometryTools::descalePoint(goalNode.origin(), m_min_x, m_min_y, m_resolution));
			path.push_back(GeometryTools::descalePoint(startNode.origin(), m_min_x, m_min_y, m_resolution));

			path = prunePath(path);
			if (m_bLog)
			{
				Planner::logPath(_logfile, path);
				_logfile.close();
			}
			return path;
		}

		if (m_bLog)
		{
			std::ostringstream oss;
			oss << "./ASTAR" << m_logNumber << ".log";
			_logfile = std::ofstream(oss.str());

			_logfile << "FIELD        (" << m_rectField.left() << "," << m_rectField.bottom() << ")->(" << m_rectField.right() << "," << m_rectField.top() << ")" << endl;
			_logfile << "RESOLUTION   (" << m_resolution << ")" << endl;
			_logfile << "SN: ORIGIN   (" << startNode.x() << "," << startNode.y() << ")" << endl;
			_logfile << "GN: ORIGIN   (" << goalNode.x() << "," << goalNode.y() << ")" << endl;

			aRectangle rc;
			for (size_t o = 0; o < m_obstacles.size(); o++)
			{
				rc = m_obstacles[o];
				_logfile << "OBS " << o << "        (" << rc.left() << "," << rc.bottom() << ")->(" << rc.right() << "," << rc.top() << ")" << endl;
			}
		}

		/**
		Calculates the <startNode> index in grid and insert the node into the Open Set.
		*/
		c_id = calcGridIndex(startNode);
		m_openSet[c_id] = startNode;
		if (m_bLog)
			_logfile << "INDEXOF: START (" << c_id << ")" << endl;

		/**
		Loops until the Open Set is not empty.
		*/
		while (m_openSet.size() > 0)
		{
			/**
			Gets the Open Set entry with lowest cost.
			Calcs the heuristic distance.
			*/
			int32_t c_id = getOpenSetLowestCostIndex(goalNode);
			if (-1 == c_id)
			{
				break;
			}
			currentNode = m_openSet[c_id];
			double dist = calcHeuristic(goalNode, currentNode);
			if (m_bLog)
				_logfile << "CN: ORIGIN   (" << currentNode.x() << "," << currentNode.y() << ") dist (" << dist << ") ix (" << c_id << ")" << endl;

			/**
			Verifies if the goal has been reached.
			*/
			if ((currentNode.x() == goalNode.x()) && (currentNode.y() == goalNode.y()) || (dist <= m_heuristicWeight))
			{
				if (m_bLog)
					_logfile << "GOAL REACHED AT COST (" << currentNode.cost() << ")" << endl;
				goalNode.setParentIndex(currentNode.parentIndex());
				goalNode.setCost(currentNode.cost());
				break;
			}

			/** Remove the item from the open set */
			m_openSet.erase(c_id);

			/** Add it to the closed set */
			m_closedSet[c_id] = currentNode;

			/** Sets up all motion nodes */
			for (uint32_t m = 0; m < m_motion.size(); m++)
			{
				node = AStarNode(
					aPoint(
						currentNode.x() + m_motion[m].x(),
						currentNode.y() + m_motion[m].y()
					),
					currentNode.cost() + m_motion[m].cost(),
					c_id
				);
				int32_t n_id = calcGridIndex(node);
				if (m_bLog)
					_logfile << "          -> (" << node.x() << "," << node.y() << ") cost (" << node.cost() << ") ix (" << n_id << ") ";

				/** If the node is not safe, do nothing */
				if (!verifyNode(node, currentNode))
				{
					if (m_bLog)
						_logfile << "NOT SAFE" << endl;
					continue;
				}

				nodeMap::iterator itc;
				itc = m_closedSet.find(n_id);
				if (itc != m_closedSet.end())
				{
					if (m_bLog)
						_logfile << "ALREADY IN CLOSED SET" << endl;
					continue;
				}

				nodeMap::iterator ito;
				ito = m_openSet.find(n_id);
				if (ito == m_openSet.end())
					m_openSet[n_id] = node;
				else
				{
					if (m_openSet[n_id].cost() > node.cost())
						m_openSet[n_id] = node;
				}
				if (m_bLog)
					_logfile << "ADDED TO OPEN SET" << endl;
			}
		}
		if (m_bLog)
		{
			_logfile << "OS: (" << m_openSet.size() << ") items" << endl;
			_logfile << "CS: (" << m_closedSet.size() << ") items" << endl;
			_logfile.close();
		}

		path = calcFinalPath(goalNode);
		return prunePath(path);
	}

	//----------------------------------------------------------------------------------------------

	pointVec AStarPlanner::plan(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, double resolution, double heuristicWeight)
	{
		init(ptStart, ptGoal, obstacles, rectField, resolution, heuristicWeight);
		return plan();
	}

}
