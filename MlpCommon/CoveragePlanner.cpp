#include "CoveragePlanner.h"

using namespace std;
//using namespace boost::numeric::ublas;

#define	ADJ_BEFORE

namespace afarcloud
{
	/**
		Coverage Planner constructors
	*/
	CoveragePlanner::CoveragePlanner()
	{
		setHelperStrings();
		setMotion();
		m_bSetUp = false;
	}

	/**
	CoveragePlanner::CoveragePlanner(aPoint ptStart, aPoint ptEnd, rectangleVec obstacles, aRectangle rectField, aPoint ptHome, double resolution)
	Summary: Coverage Planner constructor.
	Params: ptStart			aPoint			See Planner constructor.
			ptGoal			aPoint			See Planner constructor.
			obstacles		rectangleVec	See Planner constructor.
			rectField		aRectangle		See Planner constructor.
			ptHome			aPoint			Home point.
			resolution		double			Algorithm's resolution.
	Return: none
	*/
	CoveragePlanner::CoveragePlanner(aPoint ptStart, aPoint ptEnd, rectangleVec obstacles, aRectangle rectField, aPoint ptHome, double resolution)
	{
		setHelperStrings();
		setMotion();
		setUp(ptStart, ptEnd, obstacles, rectField, ptHome, resolution);
	}

	CoveragePlanner::~CoveragePlanner()
	{
	}

	/**
	void CoveragePlanner::setMotion()
	Summary: Sets up the motion vector. It fills the <m_motion> vector with all possible moves.
	Params: none.
	Return:	none.
	*/
	void CoveragePlanner::setMotion()
	{
		m_motion.push_back(BoustrophedonMotion(-1, 0));
		//m_motion.push_back(BoustrophedonMotion(-1, 1));
		m_motion.push_back(BoustrophedonMotion(0, 1));
		//m_motion.push_back(BoustrophedonMotion(1, 1));
		m_motion.push_back(BoustrophedonMotion(1, 0));
		//m_motion.push_back(BoustrophedonMotion(1, -1));
		m_motion.push_back(BoustrophedonMotion(0, -1));
		//m_motion.push_back(BoustrophedonMotion(-1, -1));
	}

	void CoveragePlanner::setHelperStrings()
	{
		m_alignmentStr.push_back("NONE");
		m_alignmentStr.push_back("TOP");
		m_alignmentStr.push_back("RIGHT");
		m_alignmentStr.push_back("BOTTOM");
		m_alignmentStr.push_back("LEFT");
		
		m_joinStatusStr.push_back("JOINT");
		m_joinStatusStr.push_back("DISJOINT");

		m_pointStr.push_back("UNSET");
		m_pointStr.push_back("BL");
		m_pointStr.push_back("TL");
		m_pointStr.push_back("--");
		m_pointStr.push_back("TR");
		m_pointStr.push_back("--");
		m_pointStr.push_back("--");
		m_pointStr.push_back("--");
		m_pointStr.push_back("BR");

		m_procStr.push_back("RU");
		m_procStr.push_back("RU*");
		m_procStr.push_back("UR");
		m_procStr.push_back("UR*");
		m_procStr.push_back("RD");
		m_procStr.push_back("RD*");
		m_procStr.push_back("DR");
		m_procStr.push_back("DR*");
		m_procStr.push_back("LU");
		m_procStr.push_back("LU*");
		m_procStr.push_back("UL");
		m_procStr.push_back("UL*");
		m_procStr.push_back("LD");
		m_procStr.push_back("LD*");
		m_procStr.push_back("DL");
		m_procStr.push_back("DL*");

		m_expansionStr.push_back("HORIZONTAL");
		m_expansionStr.push_back("VERTICAL");
	}

	void CoveragePlanner::setUp(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField, aPoint ptHome, double resolution)
	{
		m_basicCells.clear();
		m_basicCellRectangleMap.clear();
		m_compoundCells.clear();
		m_compoundCellsH.clear();
		m_compoundCellsV.clear();
		m_obstacles.clear();
		m_xCoords.clear();
		m_yCoords.clear();
		m_forbiddenRects.clear();
		m_vertices.clear();

		//m_pointStart = ptStart;
		//m_pointEnd = ptEnd;
		m_pointHome = ptHome;

		aRectangle rc;
		vector<aRectangle>::iterator ito = obstacles.begin();
		while (ito != obstacles.end())
		{
			rc = *ito;
			m_obstacles.push_back(rc);
			ito++;
		}

		m_rectField = rectField;
		m_resolution = resolution;

		if (ptStart.x() < ptGoal.x())
		{
			//	NE
			if (ptStart.y() < ptGoal.y())
			{
				m_direction = NE_DIRECTION;
				m_pointStart = ptStart;
				m_pointEnd = ptGoal;
			}
			//	SE
			else
			{
				m_direction = SE_DIRECTION;
				m_pointStart = aPoint(ptStart.x(), ptGoal.y());
				m_pointEnd = aPoint(ptGoal.x(), ptStart.y());
			}
		}
		else
		{
			//	NW
			if (ptStart.y() < ptGoal.y())
			{
				m_direction = NW_DIRECTION;
				m_pointStart = aPoint(ptGoal.x(), ptStart.y());
				m_pointEnd = aPoint(ptStart.x(), ptGoal.y());
			}
			//	SW
			else
			{
				m_direction = SW_DIRECTION;
				m_pointStart = ptGoal;
				m_pointEnd = ptStart;
			}
		}

		m_connectionAlgorithm = 0;
		m_decompositionType = 0;
		m_bSetUp = true;
	}

	int CoveragePlanner::howManySteps(double from, double to)
	{
		int steps = 0;
		while (from <= to)
		{
			steps++;
			from += m_resolution;
		}

		return steps;
	}

	void CoveragePlanner::howManyTurnsInRectangle(aRectangle aRect, int &turnsH, int &turnsV)
	{
		turnsH = 2 * howManySteps(aRect.left(), aRect.right()) + 1;
		turnsV = 2 * howManySteps(aRect.bottom(), aRect.top()) + 1;
	}

	/**
	void CoveragePlanner::sortAscendingCoordinates(doubleVec &coord)
	Summary: Sorts ascending the vector <coords> containing a single coordinate.
	Params: coord		doubleVec		The vector of coordinates to sort.
	Return:	none.
	*/
	void CoveragePlanner::sortAscendingCoordinates(doubleVec &coord)
	{
		size_t last, k;
		bool bSwapped;
		double tmp;

		last = 1;
		do
		{
			bSwapped = false;
			k = 0;
			for (size_t xx = 0; xx < coord.size() - last; xx++)
			{
				if (coord[xx] > coord[xx + 1])
				{
					tmp = coord[xx];
					coord[xx] = coord[xx + 1];
					coord[xx + 1] = tmp;
					bSwapped = true;
					k = 0;
				}
				else
					k++;
			}
			last += k;
		} while (bSwapped);
	}

	/**
	void CoveragePlanner::sortDescendingCoordinates(doubleVec &coord)
	Summary: Sorts descending the vector <coords> containing a single coordinate.
	Params: coord		doubleVec		The vector of coordinates to sort.
	Return:	none.
	*/
	void CoveragePlanner::sortDescendingCoordinates(doubleVec &coord)
	{
		size_t last, k;
		bool bSwapped;
		double tmp;

		last = 1;
		do
		{
			bSwapped = false;
			k = 0;
			for (size_t xx = 0; xx < coord.size() - last; xx++)
			{
				if (coord[xx] < coord[xx + 1])
				{
					tmp = coord[xx];
					coord[xx] = coord[xx + 1];
					coord[xx + 1] = tmp;
					bSwapped = true;
					k = 0;
				}
				else
					k++;
			}
			last += k;
		} while (bSwapped);
	}

	/**
	double CoveragePlanner::getNearestInGridFromLesser(double v0, double v1, bool bConsiderTouch)
	Summary: Gets the nearest multiple of m_resolution, approximating v1 from v0, considering or not the equivalence
	nearest=v1.
	Params: v0				double		The initial value.
			v1				double		The final value,
			considerTouch	bool		Touch flag
	Return:	The approximate value.
	*/
	double CoveragePlanner::getNearestInGridFromLesser(double v0, double v1, bool bConsiderTouch)
	{
		if (bConsiderTouch)
		{
			while (v0 <= v1)
				v0 += m_resolution;
		}
		else
		{
			while (v0 < v1)
				v0 += m_resolution;
		}
		v0 -= m_resolution;
		return v0;
	}

	/**
	double CoveragePlanner::getNearestInGridFromGreater(double v0, double v1, bool bConsiderTouch)
	Summary: Gets the nearest multiple of m_resolution, approximating v1 from v0, considering or not the equivalence
	nearest=v1.
	Params: v0				double		The initial value.
			v1				double		The final value,
			considerTouch	bool		Touch flag
	Return:	The approximate value.
	*/
	double CoveragePlanner::getNearestInGridFromGreater(double v0, double v1, bool bConsiderTouch)
	{
		if (bConsiderTouch)
		{
			while (v0 >= v1)
				v0 -= m_resolution;
		}
		else
		{
			while (v0 > v1)
				v0 -= m_resolution;
		}
		v0 += m_resolution;
		return v0;
	}

	bool CoveragePlanner::isPointInsideAnObstacle(aPoint pt)
	{
		for (size_t na = 0; na < m_obstacles.size(); na++)
			if (m_obstacles[na].contains(pt, false))
				return true;
		return false;
	}

	bool CoveragePlanner::intersectsAnObstacle(aRectangle ar)
	{
		for (size_t na = 0; na < m_obstacles.size(); na++)
			if (m_obstacles[na].intersect(ar, true))
				return true;
		return false;
	}

	bool CoveragePlanner::intersectsAnObstacle(aPoint pt1, aPoint pt2)
	{
		aRectangle rc;
		vector<aRectangle>::iterator ito = m_obstacles.begin();
		while (ito != m_obstacles.end())
		{
			rc = *ito;
			if (rc.intersect2(pt1, pt2))
				return true;
			ito++;
		}
		return false;
	}

	aRectangle CoveragePlanner::getObstacleIntercepted(aPoint pt1, aPoint pt2)
	{
		double x0, y0, m, x, y, dxy;
		int n, nxy;
		if (pt1.x() == pt2.x())
		{
			dxy = pt2.y() - pt1.y();
			nxy = (int)fabs(dxy / m_resolution);
			x = pt1.x();
			for (size_t na = 0; na < m_obstacles.size(); na++)
			{
				y = pt1.y();
				for (n = 0; n < nxy; n++)
				{
					if (m_obstacles[na].contains(aPoint(x, y)))
						return m_obstacles[na];
					y += m_resolution;
				}
			}
		}
		else
		{
			x0 = pt1.x();
			y0 = pt1.y();
			dxy = pt2.x() - x0;
			m = (pt2.y() - y0) / dxy;
			nxy = (int)fabs(dxy / m_resolution);
			for (size_t na = 0; na < m_obstacles.size(); na++)
			{
				x = pt1.x();
				for (n = 0; n < nxy; n++)
				{
					y = y0 + (x - x0)*m;
					if (m_obstacles[na].contains(aPoint(x, y)))
						return m_obstacles[na];
					x += m_resolution;
				}
			}
		}

		return aRectangle::nullRect;
	}

	/**
	void CoveragePlanner::createBasicCellStatusMap()
	Summary: Fills the map <m_basicCellStatusMap> Sets up the motion vector. It fills the <m_motion> vector with all possible moves.
	Params: none.
	Return:	none.
	*/
	void CoveragePlanner::createBasicCellStatusMap()
	{
		int tag;
		size_t rMax = m_yCoords.size() - 1;		///	Rectangles are 1 less then the coordinates !
		size_t cMax = m_xCoords.size() - 1;
		/**
		Each cell is considered as a rectangle whose coordinates are:
		First corner:	m_xCoords[c],m_yCoords[r]
		Second corner:	m_xCoords[c+1],m_yCoords[r+1]
		*/
		/**
		Maps all cells as forbidden
		*/
		for (size_t r = 0; r < rMax; r++)
			for (size_t c = 0; c < cMax; c++)
				m_basicCellStatusMap[makeTag(r, c)] = CELL_STATUS_FORBIDDEN;	//	Forbidden
		/**
		Maps all allowed cells, contained in <m_basicCells>, as unexplored, using the rectangle tag to identify the cell.
		*/
		for (size_t n = 0; n < m_basicCells.size(); n++)
		{
			tag = m_basicCells[n].getTag();
			m_basicCellStatusMap[tag] = CELL_STATUS_UNEXPLORED;	//	unexplored
		}
	}

	/**
	pointVec CoveragePlanner::prunePath(pointVec path, int &turns)
	Summary: Prunes the <path> deleting the points belonging to the same line.
	Params: path			pointVec		The path to prune.
	Return:					pointVec		The pruned path.
	*/
	pointVec CoveragePlanner::prunePath(pointVec path, int &turns)
	{
		turns = 0;
		aPoint pt0, pt1, pt2;
		bool bBelongs;
		pointVec outPath;

		pt0 = path[0];
		outPath.push_back(pt0);
		pt1 = path[1];
		for (size_t n = 2; n < path.size(); n++)
		{
			pt2 = path[n];
			bBelongs = GeometryTools::belongsToLine(pt0, pt1, pt2);
			if (bBelongs && (pt0.x() != pt2.x()) && (pt0.y() != pt2.y()))
			{
				pt1 = pt2;
			}
			else
			{
				outPath.push_back(pt1);
				pt0 = pt1;
				pt1 = pt2;
				turns++;
			}
		}
		outPath.push_back(pt2);

		return outPath;
	}

	void CoveragePlanner::calcCoordinatesWithResolution()
	{
		aRectangle o;
		double l, r, b, t;
		double lf, rf, bf, tf;
		int mask;

		aPoint a0 = m_pointStart;
		aPoint a1 = m_pointEnd;

		lf = a0.x();
		bf = a0.y();
		rf = getNearestInGridFromLesser(a0.x(), a1.x(), false) + m_resolution;
		tf = getNearestInGridFromLesser(a0.y(), a1.y(), false) + m_resolution;

		m_xCoords.push_back(lf);
		m_xCoords.push_back(rf);
		m_yCoords.push_back(bf);
		m_yCoords.push_back(tf);

		aRectangle aRect(aPoint(lf, bf), aPoint(rf, tf));

		for (size_t na = 0; na < m_obstacles.size(); na++)
		{
			o = m_obstacles[na];
			if (aRect.contains(o))
			{
				l = getNearestInGridFromLesser(lf, o.left());
				if (find(m_xCoords.begin(), m_xCoords.end(), l) == m_xCoords.end())
					m_xCoords.push_back(l);
				r = getNearestInGridFromGreater(rf, o.right());
				if (find(m_xCoords.begin(), m_xCoords.end(), r) == m_xCoords.end())
					m_xCoords.push_back(r);
				b = getNearestInGridFromLesser(bf, o.bottom());
				if (find(m_yCoords.begin(), m_yCoords.end(), b) == m_yCoords.end())
					m_yCoords.push_back(b);
				t = getNearestInGridFromGreater(tf, o.top());
				if (find(m_yCoords.begin(), m_yCoords.end(), t) == m_yCoords.end())
					m_yCoords.push_back(t);
				m_forbiddenRects.push_back(aRectangle(aPoint(l, b), aPoint(r, t)));
			}
			else// if (aRect.intersect(m_obstacles[na]))
			{
				mask = 0;
				if (aRect.contains(o.bottomLeft()))
				{
					l = getNearestInGridFromLesser(lf, o.left());
					if (find(m_xCoords.begin(), m_xCoords.end(), l) == m_xCoords.end())
						m_xCoords.push_back(l);
					b = getNearestInGridFromLesser(bf, o.bottom());
					if (find(m_yCoords.begin(), m_yCoords.end(), b) == m_yCoords.end())
						m_yCoords.push_back(b);
					mask |= 0x0001;
				}
				if (aRect.contains(o.topLeft()))
				{
					l = getNearestInGridFromLesser(lf, o.left());
					if (find(m_xCoords.begin(), m_xCoords.end(), l) == m_xCoords.end())
						m_xCoords.push_back(l);
					t = getNearestInGridFromGreater(tf, o.top());
					if (find(m_yCoords.begin(), m_yCoords.end(), t) == m_yCoords.end())
						m_yCoords.push_back(t);
					mask |= 0x0004;
				}
				if (aRect.contains(o.bottomRight()))
				{
					r = getNearestInGridFromGreater(rf, o.right());
					if (find(m_xCoords.begin(), m_xCoords.end(), r) == m_xCoords.end())
						m_xCoords.push_back(r);
					b = getNearestInGridFromLesser(bf, o.bottom());
					if (find(m_yCoords.begin(), m_yCoords.end(), b) == m_yCoords.end())
						m_yCoords.push_back(b);
					mask |= 0x0002;
				}
				if (aRect.contains(o.topRight()))
				{
					r = getNearestInGridFromGreater(rf, o.right());
					if (find(m_xCoords.begin(), m_xCoords.end(), r) == m_xCoords.end())
						m_xCoords.push_back(r);
					t = getNearestInGridFromGreater(tf, o.top());
					if (find(m_yCoords.begin(), m_yCoords.end(), t) == m_yCoords.end())
						m_yCoords.push_back(t);
					mask |= 0x0008;
				}
				switch (mask)
				{
				case 0x0001: m_forbiddenRects.push_back(aRectangle(aPoint(l, b), a1)); break;						//	BL
				case 0x0002: m_forbiddenRects.push_back(aRectangle(aPoint(lf, b), aPoint(r, tf))); break;	//	BR
				case 0x0003: m_forbiddenRects.push_back(aRectangle(aPoint(l, b), aPoint(r, tf))); break;		//	BL/BR
				case 0x0004: m_forbiddenRects.push_back(aRectangle(aPoint(l, bf), aPoint(lf, t))); break;	//	TL
				case 0x0005: m_forbiddenRects.push_back(aRectangle(aPoint(l, b), aPoint(rf, t))); break;		//	TL/BL
				case 0x0006: break;																				//	TL/BR
				case 0x0007: break;																				//	TL/BR/BL
				case 0x0008: m_forbiddenRects.push_back(aRectangle(a0, aPoint(r, t))); break;						//	TR
				case 0x0009: break;																				//	TR/BL
				case 0x000A: m_forbiddenRects.push_back(aRectangle(aPoint(lf, b), aPoint(r, t))); break;		//	TR/BR
				case 0x000B: break;																				//	TR/BR/BL
				case 0x000C: m_forbiddenRects.push_back(aRectangle(aPoint(l, bf), aPoint(r, t))); break;		//	TL/TR
				case 0x000D: break;																				//	TR/TL/BL
				case 0x000E: break;																				//	TR/TL/BR
				case 0x000F: break;																				//	TR/BR/TL/BL
				}
			}
		}
	}

	void CoveragePlanner::calcCoordinatesWithoutResolution()
	{
		aRectangle o;
		double l, r, b, t;
		double lf, rf, bf, tf;
		int mask;

		aPoint a0 = m_pointStart;
		aPoint a1 = m_pointEnd;

		lf = a0.x();
		bf = a0.y();
		rf = a1.x();
		tf = a1.y();

		m_xCoords.push_back(lf);
		m_xCoords.push_back(rf);
		m_yCoords.push_back(bf);
		m_yCoords.push_back(tf);

		aRectangle aRect(aPoint(lf, bf), aPoint(rf, tf));

		for (size_t na = 0; na < m_obstacles.size(); na++)
		{
			o = m_obstacles[na];
			if (aRect.contains(o))
			{
				l = o.left();
				if (find(m_xCoords.begin(), m_xCoords.end(), l) == m_xCoords.end())
					m_xCoords.push_back(l);
				r = o.right();
				if (find(m_xCoords.begin(), m_xCoords.end(), r) == m_xCoords.end())
					m_xCoords.push_back(r);
				b = o.bottom();
				if (find(m_yCoords.begin(), m_yCoords.end(), b) == m_yCoords.end())
					m_yCoords.push_back(b);
				t = o.top();
				if (find(m_yCoords.begin(), m_yCoords.end(), t) == m_yCoords.end())
					m_yCoords.push_back(t);
				m_forbiddenRects.push_back(aRectangle(aPoint(l, b), aPoint(r, t)));
			}
			else// if (aRect.intersect(m_obstacles[na]))
			{
				mask = 0;
				if (aRect.contains(o.bottomLeft()))
				{
					l = o.left();
					if (find(m_xCoords.begin(), m_xCoords.end(), l) == m_xCoords.end())
						m_xCoords.push_back(l);
					b = o.bottom();
					if (find(m_yCoords.begin(), m_yCoords.end(), b) == m_yCoords.end())
						m_yCoords.push_back(b);
					mask |= 0x0001;
				}
				if (aRect.contains(o.topLeft()))
				{
					l = o.left();
					if (find(m_xCoords.begin(), m_xCoords.end(), l) == m_xCoords.end())
						m_xCoords.push_back(l);
					t = o.top();
					if (find(m_yCoords.begin(), m_yCoords.end(), t) == m_yCoords.end())
						m_yCoords.push_back(t);
					mask |= 0x0004;
				}
				if (aRect.contains(o.bottomRight()))
				{
					r = o.right();
					if (find(m_xCoords.begin(), m_xCoords.end(), r) == m_xCoords.end())
						m_xCoords.push_back(r);
					b = o.bottom();
					if (find(m_yCoords.begin(), m_yCoords.end(), b) == m_yCoords.end())
						m_yCoords.push_back(b);
					mask |= 0x0002;
				}
				if (aRect.contains(o.topRight()))
				{
					r = o.right();
					if (find(m_xCoords.begin(), m_xCoords.end(), r) == m_xCoords.end())
						m_xCoords.push_back(r);
					t = o.top();
					if (find(m_yCoords.begin(), m_yCoords.end(), t) == m_yCoords.end())
						m_yCoords.push_back(t);
					mask |= 0x0008;
				}
				switch (mask)
				{
				case 0x0001: m_forbiddenRects.push_back(aRectangle(aPoint(l, b), a1)); break;						//	BL
				case 0x0002: m_forbiddenRects.push_back(aRectangle(aPoint(lf, b), aPoint(r, tf))); break;	//	BR
				case 0x0003: m_forbiddenRects.push_back(aRectangle(aPoint(l, b), aPoint(r, tf))); break;		//	BL/BR
				case 0x0004: m_forbiddenRects.push_back(aRectangle(aPoint(l, bf), aPoint(lf, t))); break;	//	TL
				case 0x0005: m_forbiddenRects.push_back(aRectangle(aPoint(l, b), aPoint(rf, t))); break;		//	TL/BL
				case 0x0006: break;																				//	TL/BR
				case 0x0007: break;																				//	TL/BR/BL
				case 0x0008: m_forbiddenRects.push_back(aRectangle(a0, aPoint(r, t))); break;						//	TR
				case 0x0009: break;																				//	TR/BL
				case 0x000A: m_forbiddenRects.push_back(aRectangle(aPoint(lf, b), aPoint(r, t))); break;		//	TR/BR
				case 0x000B: break;																				//	TR/BR/BL
				case 0x000C: m_forbiddenRects.push_back(aRectangle(aPoint(l, bf), aPoint(r, t))); break;		//	TL/TR
				case 0x000D: break;																				//	TR/TL/BL
				case 0x000E: break;																				//	TR/TL/BR
				case 0x000F: break;																				//	TR/BR/TL/BL
				}
			}
		}
	}

	/**
	pointVec CoveragePlanner::plan()
	Summary: The coverage planner algorithm.
	Params: none.
	Return:	path		pointVec		The path obtained.
	*/
	pointVec CoveragePlanner::plan()
	{
		vector<aPoint> path;

		/**
		Initialization check.
		*/
		if (!m_bSetUp)
			return path;

		aRectangle o;

		/**
		Calcs obstacles and coverage area coordinates (grid aligned)
		*/

		//calcCoordinatesWithResolution();
		calcCoordinatesWithoutResolution();

		/**
		Sort obtained coordinates
		*/

		switch (m_direction)
		{
		case NE_DIRECTION:
			sortAscendingCoordinates(m_xCoords);
			sortAscendingCoordinates(m_yCoords);
			break;
		case SE_DIRECTION:
			sortAscendingCoordinates(m_xCoords);
			sortDescendingCoordinates(m_yCoords);
			break;
		case NW_DIRECTION:
			sortDescendingCoordinates(m_xCoords);
			sortAscendingCoordinates(m_yCoords);
			break;
		case SW_DIRECTION:
			sortDescendingCoordinates(m_xCoords);
			sortDescendingCoordinates(m_yCoords);
			break;
		}

		/**
		Calcs all allowed vertices
		*/
		aRectangle cell;
		aPoint ap;
		for (size_t yy = 0; yy < m_yCoords.size(); yy++)
			for (size_t xx = 0; xx < m_xCoords.size(); xx++)
			{
				ap = aPoint(m_xCoords[xx], m_yCoords[yy]);
				if (!isPointInsideAnObstacle(ap))
				{
					m_vertices.push_back(ap);
				}
			}

		/**
		Calc all basic cells.
		Each cell is tagged rrrrcccc
		*/
		aPoint aCenter;
		int tag;
		for (size_t yy = 0; yy < m_yCoords.size() - 1; yy++)
			for (size_t xx = 0; xx < m_xCoords.size() - 1; xx++)
			{
				cell = aRectangle(aPoint(m_xCoords[xx], m_yCoords[yy]), aPoint(m_xCoords[xx + 1], m_yCoords[yy + 1]));
				aCenter = cell.center();
				if (!isPointInsideAnObstacle(aCenter)/* && !intersectsAnObstacle(cell)*/)
				{
					tag = makeTag(yy, xx);
					cell.setTag(tag);
					m_basicCells.push_back(cell);
				}
			}
		
		/**
		Create a map of rectangles tag->rectangle
		*/
		rectangleVec::iterator it = m_basicCells.begin();
		while (it != m_basicCells.end())
		{
			cell = *it;
			m_basicCellRectangleMap[cell.getTag()] = cell;
			it++;
		}

		pointVec fullPath;

		m_logfile = std::ofstream("./strategy.log");

		/**
		Basic cell map creation
		*/
		createBasicCellStatusMap();

		/**
		The coverage planner kernel
		*/
		switch (m_decompositionType)
		{
		default:
		case 0:
			fullPath = strategyBoustrophedon();
			break;
		case 1:
			fullPath = strategyAggregation();
			break;
		}

		Planner::logPath(m_logfile, fullPath);

		pointVec::iterator ip = fullPath.begin();
		aPoint pt;
		while (ip != fullPath.end())
		{
			//pt = aPoint(ip->x() + m_pointStart.x(), ip->y() + m_pointStart.y());
			pt = *ip;
			path.push_back(pt);
			ip++;
		}
		Planner::logPath(m_logfile, path);

		m_logfile.close();

		return path;
	}


}
