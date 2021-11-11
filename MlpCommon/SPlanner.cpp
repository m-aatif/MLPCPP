#include "SPlanner.h"
#include "Configurator.h"

using namespace std;

namespace afarcloud
{
	int SPlanner::m_logNumber = 0;

	/**
		Coverage Planner constructors
	*/
	SPlanner::SPlanner()
	{
		m_bSetUp = false;
	}

	SPlanner::SPlanner(aPoint ptStart, aPoint ptEnd, rectangleVec obstacles, aRectangle rectField, double resolution)
	{
		init(ptStart, ptEnd, obstacles, rectField, resolution);
	}

	SPlanner::~SPlanner()
	{
	}

	void SPlanner::init(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, double resolution)
	{
		m_pointStart = ptStart;
		m_pointGoal = ptGoal;
		m_resolution = resolution;

		m_min_x = rectField.left();
		m_min_y = rectField.bottom();

		m_rectField = GeometryTools::scaleRect(rectField, m_min_x, m_min_y, m_resolution);

		m_max_x = rectField.right();
		m_max_y = rectField.top();
		m_width = (m_max_x - m_min_x) / m_resolution;
		m_height = (m_max_y - m_min_y) / m_resolution;
		m_max_x = m_min_x + m_width;
		m_max_y = m_min_y + m_height;

		aRectangle rcObs, rcObsBound, rcx;
		rectangleVec::iterator ito = obstacles.begin();
		while (ito != obstacles.end())
		{
			rcObs = *ito;
			rcx = GeometryTools::scaleRect(rcObs, m_min_x, m_min_y, m_resolution);
			m_obstacles.push_back(rcx);

			rcObsBound = rcObs;
			rcObsBound.inflate(m_resolution, m_resolution, m_resolution, m_resolution);
			m_obsVertices.push_back(GeometryTools::scalePoint(rcObsBound.bottomLeft(), m_min_x, m_min_y, m_resolution));
			m_obsVertices.push_back(GeometryTools::scalePoint(rcObsBound.topLeft(), m_min_x, m_min_y, m_resolution));
			m_obsVertices.push_back(GeometryTools::scalePoint(rcObsBound.topRight(), m_min_x, m_min_y, m_resolution));
			m_obsVertices.push_back(GeometryTools::scalePoint(rcObsBound.bottomRight(), m_min_x, m_min_y, m_resolution));

			ito++;
		}
		m_obsVertices.push_back(GeometryTools::scalePoint(m_pointGoal, m_min_x, m_min_y, m_resolution));

		m_bSetUp = true;
	}

	int32_t SPlanner::calcGridIndex(SNode node)
	{
		return (int32_t)(node.y() * m_rectField.width() + node.x());
	}

	bool SPlanner::collides(SNode baseNode, SNode nearNode)
	{
		aRectangle rc;
		rectangleVec::iterator ito = m_obstacles.begin();
		while (ito != m_obstacles.end())
		{
			rc = *ito;
			if (rc.intersect2(baseNode.origin(), nearNode.origin()))
			{
				return true;
			}
			ito++;
		}
		return false;
	}

	bool SPlanner::collides(aPoint p1, aPoint p2)
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

	int32_t SPlanner::getOpenSetLowestCostIndex(SNode goalNode)
	{
		int32_t retVal = -1;
		double minCost = 1.0E10, cost;
		snodeMap::iterator it = m_openSet.begin();
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

	double SPlanner::calcHeuristic(SNode node1, SNode node2)
	{
		return 1.0 * hypot(node1.x() - node2.x(), node1.y() - node2.y());
	}

	pointVec SPlanner::plan()
	{
		pointVec path;

		if (!m_bSetUp)
			return path;

		SNode node, currentNode;
		SNode startNode(GeometryTools::scalePoint(m_pointStart, m_min_x, m_min_y, m_resolution), 0.0, -1);
		SNode goalNode(GeometryTools::scalePoint(m_pointGoal, m_min_x, m_min_y, m_resolution), 0.0, -1);

		int gridIndex;
		std::ofstream _logfile;

		if (m_bLog)
		{
			std::ostringstream oss;
			oss << Configurator::getLogFolder() << "\\SPLANNER" << m_logNumber << ".log";
			_logfile = std::ofstream(oss.str());
		}

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
			aPoint pt;
			for (size_t v = 0; v < m_obsVertices.size(); v++)
			{
				pt = m_obsVertices[v];
				_logfile << "OVX " << v << "        (" << pt.x() << "," << pt.y() << ")" << endl;
			}
		}

		gridIndex = calcGridIndex(startNode);
		m_openSet[gridIndex] = startNode;
		if (m_bLog)
			_logfile << "INDEXOF: START (" << gridIndex << ")" << endl;

		map<uint32_t, uint32_t> visitedVertices;

		while (m_openSet.size() > 0)
		{
			int32_t c_id = getOpenSetLowestCostIndex(goalNode);

			currentNode = m_openSet[c_id];
			if (m_bLog)
				_logfile << "CURRENT NODE: ORIGIN (" << currentNode.x() << "," << currentNode.y() << ") Cost " << currentNode.cost() << ". ParentIndex " << currentNode.parentIndex() << endl;

			aPoint currentNodePos = currentNode.origin();
			aPoint goalNodePos = goalNode.origin();
			if ((currentNodePos.x() == goalNodePos.x()) && (currentNodePos.y() == goalNodePos.y()))
			{
				goalNode.setParentIndex(currentNode.parentIndex());
				goalNode.setCost(currentNode.cost());
				if (m_bLog)
					_logfile << "GOAL    NODE: ORIGIN (" << goalNode.x() << "," << goalNode.y() << ") Cost " << currentNode.cost() << ". REACHED !" << endl;
				break;
			}

			/** Remove the item from the open set */
			if (m_bLog)
				_logfile << "ERASING NODE (" << c_id << ") from OPEN SET" << endl;
			m_openSet.erase(c_id);

			/** Add it to the closed set */
			if (m_bLog)
				_logfile << "ADDING  NODE (" << c_id << ") to CLOSED SET" << endl;
			m_closedSet[c_id] = currentNode;

			for (uint32_t v = 0; v < m_obsVertices.size(); v++)
			{
				if (m_bLog)
					_logfile << "TRYING VERTEX(" << v << "): (" << m_obsVertices[v].x() << "," << m_obsVertices[v].y() << ")";
				map<uint32_t, uint32_t>::iterator vvi = visitedVertices.find(v);
				if (vvi != visitedVertices.end())
				{
					if (m_bLog)
						_logfile << " already VISITED" << endl;
					continue;
				}
				if (m_bLog)
					_logfile << " FREE" << endl;

				node = SNode(
					m_obsVertices[v],
					0.0,
					c_id
				);
				node.setCost(currentNode.cost() + calcHeuristic(currentNode, node));
				if (m_bLog)
					_logfile << "        NODE: ORIGIN (" << node.x() << "," << node.y() << ") Cost " << node.cost() << ". ParentIndex " << node.parentIndex() << endl;
				int32_t n_id = calcGridIndex(node);

				/** If the node is not safe, do nothing */
				if (!verifyNode(node, currentNode))
				{
					if (m_bLog)
						_logfile << "CURRENT NODE and NODE: ORIGIN (" << node.x() << "," << node.y() << ") Cost " << node.cost() << ". ParentIndex " << node.parentIndex() << ". COLLIDE !" << endl;
					continue;
				}

				snodeMap::iterator itc;
				itc = m_closedSet.find(n_id);
				if (itc != m_closedSet.end())
				{
					if (m_bLog)
						_logfile << "        NODE: ORIGIN (" << node.x() << "," << node.y() << ") Cost " << node.cost() << ". ParentIndex " << node.parentIndex() << ". Already in CLOSED SET !" << endl;
					continue;
				}

				snodeMap::iterator ito;
				ito = m_openSet.find(n_id);
				if (ito == m_openSet.end())
				{
					if (m_bLog)
						_logfile << "ADDING  NODE(" << n_id << "): ORIGIN (" << node.x() << "," << node.y() << ") Cost " << node.cost() << ". ParentIndex " << node.parentIndex() << ". To OPEN SET !" << endl;
					m_openSet[n_id] = node;
				}
				else
				{
					if (m_openSet[n_id].cost() > node.cost())
					{
						if (m_bLog)
							_logfile << "UPDATING  NODE (" << n_id << ")" << endl;
						m_openSet[n_id] = node;
					}
				}

				if (m_bLog)
					_logfile << "SETTING NODE INDEX(" << v << ") to VISITED !" << endl;
				visitedVertices[v] = n_id;
			}
			if (m_bLog)
				_logfile << endl;
		}

		//logClosedSet(goalNode);
		path = prunePath(calcFinalPath(goalNode));
		if (m_bLog)
		{
			Planner::logPath(_logfile, path);
			_logfile.close();
		}
		return path;
	}

	pointVec SPlanner::calcFinalPath(SNode goalNode)
	{
		pointVec path;
		path.push_back(GeometryTools::descalePoint(goalNode.origin(), m_min_x, m_min_y, m_resolution));
		int32_t parentIndex = goalNode.parentIndex();
		while (parentIndex != -1)
		{
			SNode n = m_closedSet[parentIndex];
			path.push_back(GeometryTools::descalePoint(n.origin(), m_min_x, m_min_y, m_resolution));
			parentIndex = n.parentIndex();
		}
		return path;
	}

	void SPlanner::logClosedSet(SNode goalNode)
	{
		std::ofstream _logfile;
		snodeMap::iterator it;
		_logfile = std::ofstream("./ClosedSet.log");
		it = m_closedSet.begin();
		while (it != m_closedSet.end())
		{
			_logfile << it->first << ": ORIGIN (" << it->second.x() << "," << it->second.y() << ") Cost " << it->second.cost() << ". ParentIndex " << it->second.parentIndex() << endl;
			it++;
		}
		_logfile << "GOAL : ORIGIN (" << goalNode.x() << "," << goalNode.y() << ") Cost " << goalNode.cost() << ". ParentIndex " << goalNode.parentIndex() << endl;
		_logfile.close();
	}

	pointVec SPlanner::prunePath(pointVec path)
	{
		pointVec ppath;
		aPoint p1, p2;
		pointVec::iterator it = path.begin();
		bool bAdded = true;

		if (path.size() <= 2)
		{
			std::reverse(path.begin(), path.end());
			return path;
		}

		p1 = *it;
		ppath.push_back(p1);
		it++;
		while (it != path.end())
		{
			p2 = *it;
			it++;

#if 0
			bAdded = false;
			if (!collides(p1, p2))
			{
				ppath.push_back(p2);
				p1 = p2;
				bAdded = true;
			}
#else
			p1 = p2;
			bAdded = true;
			ppath.push_back(p2);
#endif // 0
		}
		if (!bAdded)
			ppath.push_back(p2);

		//----------------------------------------------------------------------
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
		std::reverse(outPath.begin(), outPath.end());

		return outPath;
		//----------------------------------------------------------------------
	}

	bool SPlanner::verifyNode(SNode node, SNode current)
	{
		aPoint nodePos = node.origin();
		aPoint basePos = m_rectField.bottomLeft();
		aPoint maxPos = m_rectField.topRight();

		if (nodePos.x() < basePos.x())
			return false;
		if (nodePos.y() < basePos.y())
			return false;
		if (nodePos.x() > maxPos.x())
			return false;
		if (nodePos.y() > maxPos.y())
			return false;

		if (collides(node, current))
			return false;

		return true;
	}

}