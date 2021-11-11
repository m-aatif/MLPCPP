#include "CoveragePlanner.h"
#include "AStarPlanner.h"
#include "SPlanner.h"

using namespace std;

namespace afarcloud
{
	/*****************************************************************************************
	* pointVec CoveragePlanner::strategyAggregation()
	* Summary: After cell decomposition, aggregates cells vertically or horizontally, depending
	*          on the minimum number of rectangles extracted.
	*          Finds the closest aggregated cell to home position.
	*          Calculates the path.
	* Params: none.
	* Return:				vector of aPoint		The path calculated.
	*****************************************************************************************/
	pointVec CoveragePlanner::strategyAggregation()
	{
		pointVec path;

		aggregateRectanglesA();

		aRectangle rc, a0, a1;
		pointVec ptx;
		coverageCellVec cellSpec;

		for (size_t nc = 0; nc < m_compoundCells.size(); nc++)
		{
			a0 = m_compoundCells[nc];
			a0.setTag(nc);
			cellSpec.push_back(CoveragePlannerCell(a0));
		}

		//---------------------------------------------------------------------
		/**
		Find closest cell to start position
		*/
		pointVec pathFromStart;
		size_t closestCellIndex = findClosestCellToStartPosition(pathFromStart, cellSpec);
		addPathToPath(path, pathFromStart);

		setCellSpecCorners(cellSpec);

		//---------------------------------------------------------------------
		logTotalCellsTurns(m_compoundCells);
		//---------------------------------------------------------------------
		//calculatePathWithResolution(cellSpec, path);
		calculatePathWithoutResolution(cellSpec, path);

		pointVec ppath = Planner::prunePath(path);
		return ppath;
	}

	/*****************************************************************************************
	* pointVec CoveragePlanner::strategyBoustrophedon()
	* Summary: After cell decomposition, aggregates cells in boustrophedon fashion.
	*          Finds the closest aggregated cell to start position.
	*          Calculates the path.
	* Params: none.
	* Return:				vector of aPoint		The path calculated.
	*****************************************************************************************/
	pointVec CoveragePlanner::strategyBoustrophedon()
	{
		pointVec path;
		cellStatusMap _cellStatusMap = m_basicCellStatusMap;

		vector<BoustrophedonCell> cellListFwd;
		vector<BoustrophedonCell> cellListBwd;

		m_logfile << "STEP 1A !" << endl << endl;
		step1fwd(m_basicCells[0].getTag(), _cellStatusMap, cellListFwd, 0);

		m_logfile << "STEP 1B !" << endl << endl;
		do
		{
			cellListBwd = cellListFwd;
			std::reverse(cellListBwd.begin(), cellListBwd.end());
			cellListFwd.clear();
			for (size_t cx = 0; cx < cellListBwd.size(); cx++)
			{
				step1fwd(cellListBwd[cx].m_tag, _cellStatusMap, cellListFwd, 0);
			}
			std::reverse(cellListFwd.begin(), cellListFwd.end());
		} while (cellListFwd.size() != m_basicCells.size());
		//-------------------------------------------
		logBoustrophedonCellList(cellListFwd);
		//-------------------------------------------
		aRectangle rcTmp;
		size_t n;
		int tag;
		for (n = 0; n < cellListFwd.size() - 1; n++)
		{
			tag = cellListFwd[n].m_tag;
			rcTmp = m_basicCellRectangleMap[tag];
			rcTmp.setTag(n);
			m_compoundCellsH.push_back(rcTmp);
		}
		//-------------------------------------------

		aRectangle rc, a0, a1;
		pointVec ptx;
		coverageCellVec cellSpec;

		tag = 0;
		bool bJoin = false;
		int dir = getBestDirectionForAggregation(0, cellListFwd);
		a0 = m_basicCellRectangleMap[cellListFwd[0].m_tag];
		for (size_t nc = 1; nc < cellListFwd.size(); nc++)
		{
			a1 = m_basicCellRectangleMap[cellListFwd[nc].m_tag];
			switch (dir)
			{
			default:
				m_logfile << "CELL INDEX " << nc << " is not aggregable" << endl;
			case N_DIRECTION:
				if (!(bJoin = a0.joinTop(a1)))
				{
					cellSpec.push_back(CoveragePlannerCell(a0, BL_CORNER, TR_CORNER));
					a0.setTag(tag++);
					m_compoundCells.push_back(a0);
					a0 = a1;
					dir = getBestDirectionForAggregation(nc, cellListFwd);
				}
				break;
			case E_DIRECTION:
				if (!(bJoin = a0.joinRight(a1)))
				{
					cellSpec.push_back(CoveragePlannerCell(a0, TL_CORNER, BR_CORNER));
					a0.setTag(tag++);
					m_compoundCells.push_back(a0);
					a0 = a1;
					dir = getBestDirectionForAggregation(nc, cellListFwd);
				}
				break;
			case S_DIRECTION:
				if (!(bJoin = a0.joinBottom(a1)))
				{
					cellSpec.push_back(CoveragePlannerCell(a0, TR_CORNER, BL_CORNER));
					a0.setTag(tag++);
					m_compoundCells.push_back(a0);
					a0 = a1;
					dir = getBestDirectionForAggregation(nc, cellListFwd);
				}
				break;
			case W_DIRECTION:
				if (!(bJoin = a0.joinLeft(a1)))
				{
					cellSpec.push_back(CoveragePlannerCell(a0, BR_CORNER, TL_CORNER));
					a0.setTag(tag++);
					m_compoundCells.push_back(a0);
					a0 = a1;
					dir = getBestDirectionForAggregation(nc, cellListFwd);
				}
				break;
			}
		}

		cellSpec.push_back(CoveragePlannerCell(a0, BL_CORNER, TR_CORNER));
		a0.setTag(tag++);
		m_compoundCells.push_back(a0);

		//---------------------------------------------------------------------
		/**
		Find closest cell to start position
		*/
		pointVec pathFromHome;
		size_t closestCellIndex = findClosestCellToStartPosition(pathFromHome, cellSpec);
		addPathToPath(path, pathFromHome);

		setCellSpecCorners(cellSpec);

		//---------------------------------------------------------------------
		logTotalCellsTurns(m_compoundCells);
		//---------------------------------------------------------------------
		//calculatePathWithResolution(cellSpec, path);
		calculatePathWithoutResolution(cellSpec, path);

		pointVec ppath = Planner::prunePath(path);
		return ppath;
	}

	void CoveragePlanner::step1fwd(int tag, map<size_t, int> &cellStatusMap, vector<BoustrophedonCell> &cellList, int lastMotionIndex)
	{
		BoustrophedonCell cell(tag);
		cellStatusMap[tag] = CELL_STATUS_EXPLORED;
		cellList.push_back(cell);

		int x, y, neighborTag, mi;
		map<size_t, int>::iterator finder;

		x = colFromTag(tag);
		y = rowFromTag(tag);

		m_logfile << "CELL (" << y << "," << x << ")";

		bool bAllNeighborsVisited = true;
		for (size_t m = 0; (m < m_motion.size()) && bAllNeighborsVisited; m++)
		{
			mi = (m + lastMotionIndex) % m_motion.size();
			neighborTag = makeTag(y + m_motion[mi].y(), x + m_motion[mi].x());
			finder = cellStatusMap.find(neighborTag);
			if (finder != cellStatusMap.end())
				if (CELL_STATUS_UNEXPLORED == cellStatusMap[neighborTag])
				{
					bAllNeighborsVisited = false;
				}
		}
		if (bAllNeighborsVisited)
			m_logfile << " has all neighbors visited !" << endl;
		else
		{
			m_logfile << " visited" << endl;
			step1fwd(neighborTag, cellStatusMap, cellList, mi);
		}
	}

	void CoveragePlanner::logBoustrophedonCellList(vector<BoustrophedonCell> cellList)
	{
		size_t n;
		int tag, x, y;
		m_logfile << endl << "CELLS (" << cellList.size() << "/" << m_basicCells.size() << ")" << endl;
		m_logfile << endl << "CELL PATH: ";
		for (n = 0; n < cellList.size() - 1; n++)
		{
			tag = cellList[n].m_tag;
			x = colFromTag(tag);
			y = rowFromTag(tag);
			m_logfile << "(" << y << "," << x << ")->";
		}
		tag = cellList[n].m_tag;
		x = colFromTag(tag);
		y = rowFromTag(tag);
		m_logfile << "(" << y << "," << x << ")" << endl << endl;
	}

	int CoveragePlanner::getBestDirectionForAggregation(size_t startCellIndex, vector<BoustrophedonCell> cellList)
	{
		aRectangle rc, a0, a1;
		pointVec ptx;
		double cellArea, macroCellArea;

		m_logfile << "AGGREGATION of CELL INDEX " << startCellIndex << endl;

		bool bJoin = false;
		int dir = NONE_DIRECTION;
		a0 = m_basicCellRectangleMap[cellList[startCellIndex].m_tag];
		macroCellArea = a0.area();
		for (size_t nc = startCellIndex + 1; nc < cellList.size(); nc++)
		{
			a1 = m_basicCellRectangleMap[cellList[nc].m_tag];
			if (!(bJoin = a0.joinTop(a1)))
			{
				m_logfile << "CELL AGGREGATION T (" << rowFromTag(cellList[startCellIndex].m_tag) << "," << rowFromTag(cellList[startCellIndex].m_tag) << ")";
				m_logfile << " -> (" << rowFromTag(cellList[nc - 1].m_tag) << "," << colFromTag(cellList[nc - 1].m_tag) << ")";
				cellArea = a0.area();
				if (cellArea > macroCellArea)
				{
					macroCellArea = cellArea;
					m_logfile << " AREA (" << macroCellArea << ")";
					dir = N_DIRECTION;
				}
				m_logfile << endl;
				break;
			}
		}
		a0 = m_basicCellRectangleMap[cellList[startCellIndex].m_tag];
		for (size_t nc = startCellIndex + 1; nc < cellList.size(); nc++)
		{
			a1 = m_basicCellRectangleMap[cellList[nc].m_tag];
			if (!(bJoin = a0.joinRight(a1)))
			{
				m_logfile << "CELL AGGREGATION R (" << rowFromTag(cellList[startCellIndex].m_tag) << "," << rowFromTag(cellList[startCellIndex].m_tag) << ")";
				m_logfile << " -> (" << rowFromTag(cellList[nc - 1].m_tag) << "," << colFromTag(cellList[nc - 1].m_tag) << ")";
				cellArea = a0.area();
				if (cellArea > macroCellArea)
				{
					macroCellArea = cellArea;
					m_logfile << " AREA (" << macroCellArea << ")";
					dir = E_DIRECTION;
				}
				m_logfile << endl;
				break;
			}
		}
		a0 = m_basicCellRectangleMap[cellList[startCellIndex].m_tag];
		for (size_t nc = startCellIndex + 1; nc < cellList.size(); nc++)
		{
			a1 = m_basicCellRectangleMap[cellList[nc].m_tag];
			if (!(bJoin = a0.joinBottom(a1)))
			{
				m_logfile << "CELL AGGREGATION B (" << rowFromTag(cellList[startCellIndex].m_tag) << "," << rowFromTag(cellList[startCellIndex].m_tag) << ")";
				m_logfile << " -> (" << rowFromTag(cellList[nc - 1].m_tag) << "," << colFromTag(cellList[nc - 1].m_tag) << ")";
				cellArea = a0.area();
				if (cellArea > macroCellArea)
				{
					macroCellArea = cellArea;
					m_logfile << " AREA (" << macroCellArea << ")";
					dir = S_DIRECTION;
				}
				m_logfile << endl;
				break;
			}
		}
		a0 = m_basicCellRectangleMap[cellList[startCellIndex].m_tag];
		for (size_t nc = startCellIndex + 1; nc < cellList.size(); nc++)
		{
			a1 = m_basicCellRectangleMap[cellList[nc].m_tag];
			if (!(bJoin = a0.joinLeft(a1)))
			{
				m_logfile << "CELL AGGREGATION L (" << rowFromTag(cellList[startCellIndex].m_tag) << "," << rowFromTag(cellList[startCellIndex].m_tag) << ")";
				m_logfile << " -> (" << rowFromTag(cellList[nc - 1].m_tag) << "," << colFromTag(cellList[nc - 1].m_tag) << ")";
				cellArea = a0.area();
				if (cellArea > macroCellArea)
				{
					macroCellArea = cellArea;
					m_logfile << " AREA (" << macroCellArea << ")";
					dir = W_DIRECTION;
				}
				m_logfile << endl;
				break;
			}
		}

		m_logfile << "FINAL DIRECTION " << dir << endl;

		return dir;
	}

	void CoveragePlanner::setCellSpecCorners(coverageCellVec &cellSpec)
	{
		if (cellSpec.size() == 1)
		{
			switch (m_direction)
			{
			case NE_DIRECTION:
				cellSpec[0].m_startCorner = BL_CORNER;
				cellSpec[0].m_endCorner = TR_CORNER;
				break;
			case NW_DIRECTION:
				cellSpec[0].m_startCorner = BR_CORNER;
				cellSpec[0].m_endCorner = TL_CORNER;
				break;
			case SW_DIRECTION:
				cellSpec[0].m_startCorner = TR_CORNER;
				cellSpec[0].m_endCorner = BL_CORNER;
				break;
			case SE_DIRECTION:
				cellSpec[0].m_startCorner = TL_CORNER;
				cellSpec[0].m_endCorner = BR_CORNER;
				break;
			}
		}
		else
		{
			for (size_t cs = 1; cs < cellSpec.size(); cs++)
			{
				if (cellSpec[cs - 1].m_rect.dockableAtRight(cellSpec[cs].m_rect))
				{
					if (cellSpec[cs - 1].m_rect.alignedTop(cellSpec[cs].m_rect))
					{
						cellSpec[cs - 1].m_endCorner = TR_CORNER;
						cellSpec[cs].m_startCorner = TL_CORNER;
					}
					else
					{
						cellSpec[cs - 1].m_endCorner = BR_CORNER;
						cellSpec[cs].m_startCorner = BL_CORNER;
					}
				}
				else if (cellSpec[cs - 1].m_rect.dockableAtBottom(cellSpec[cs].m_rect))
				{
					if (cellSpec[cs - 1].m_rect.alignedLeft(cellSpec[cs].m_rect))
					{
						cellSpec[cs - 1].m_endCorner = BL_CORNER;
						cellSpec[cs].m_startCorner = TL_CORNER;
					}
					else
					{
						cellSpec[cs - 1].m_endCorner = BR_CORNER;
						cellSpec[cs].m_startCorner = TR_CORNER;
					}
				}
				else if (cellSpec[cs - 1].m_rect.dockableAtLeft(cellSpec[cs].m_rect))
				{
					if (cellSpec[cs - 1].m_rect.alignedTop(cellSpec[cs].m_rect))
					{
						cellSpec[cs - 1].m_endCorner = TL_CORNER;
						cellSpec[cs].m_startCorner = TR_CORNER;
					}
					else
					{
						cellSpec[cs - 1].m_endCorner = BL_CORNER;
						cellSpec[cs].m_startCorner = BR_CORNER;
					}
				}
				else if (cellSpec[cs - 1].m_rect.dockableAtTop(cellSpec[cs].m_rect))
				{
					if (cellSpec[cs - 1].m_rect.alignedLeft(cellSpec[cs].m_rect))
					{
						cellSpec[cs - 1].m_endCorner = TL_CORNER;
						cellSpec[cs].m_startCorner = BL_CORNER;
					}
					else
					{
						cellSpec[cs - 1].m_endCorner = TR_CORNER;
						cellSpec[cs].m_startCorner = BR_CORNER;
					}
				}
			}
		}
	}

	/**
		SUPPORT FUNCTIONS
	*/
	pointVec CoveragePlanner::pathPlan(aPoint ptStart, aPoint ptEnd)
	{
		Planner *pPlanner = NULL;
		pointVec path;

		if (0 == m_connectionAlgorithm)
		{
			pPlanner = new AStarPlanner(ptStart, ptEnd, m_obstacles, m_rectField, m_connectionAlgorithmResolution, m_connectionAlgorithmExtraParam1);
		}
		else if (3 == m_connectionAlgorithm)
		{
			pPlanner = new SPlanner(ptStart, ptEnd, m_obstacles, m_rectField, m_connectionAlgorithmResolution);
		}
		if (NULL != pPlanner)
		{
			path = pPlanner->plan();
			delete pPlanner;
		}
		return path;
	}

	size_t CoveragePlanner::findClosestCellToStartPosition(pointVec &path, coverageCellVec &cells)
	{
		aRectangle rc;
		aPoint closestPoint;
		size_t closestCellIndex = 0;
		double d, minDist = 1.0E6;
		int startCorner = BL_CORNER;
		for (size_t bc = 0; bc < cells.size(); bc++)
		{
			rc = cells[bc].m_rect;
			d = hypot(m_pointHome.x() - rc.left(), m_pointHome.y() - rc.bottom());
			if (d < minDist)
			{
				minDist = d;
				closestCellIndex = bc;
				closestPoint = rc.bottomLeft();
				startCorner = BL_CORNER;
			}
			d = hypot(m_pointHome.x() - rc.left(), m_pointHome.y() - rc.top());
			if (d < minDist)
			{
				minDist = d;
				closestCellIndex = bc;
				closestPoint = rc.topLeft();
				startCorner = TL_CORNER;
			}
			d = hypot(m_pointHome.x() - rc.right(), m_pointHome.y() - rc.bottom());
			if (d < minDist)
			{
				minDist = d;
				closestCellIndex = bc;
				closestPoint = rc.bottomRight();
				startCorner = BR_CORNER;
			}
			d = hypot(m_pointHome.x() - rc.right(), m_pointHome.y() - rc.top());
			if (d < minDist)
			{
				minDist = d;
				closestCellIndex = bc;
				closestPoint = rc.topRight();
				startCorner = TR_CORNER;
			}
		}

		path = pathPlan(m_pointHome, closestPoint);

		return closestCellIndex;
	}

	void CoveragePlanner::calculatePathWithResolution(coverageCellVec cellSpec, pointVec &path)
	{
		int turnsH, turnsV;
		pointVec linkPath;
		aPoint startPoint;
		aPoint endPoint;
		aRectangle rc;
		pointVec ptx;

		m_logfile << "START (" << m_pointStart.x() << "," << m_pointStart.y() << ")" << endl;
		m_logfile << "END   (" << m_pointEnd.x() << "," << m_pointEnd.y() << ")" << endl;

		for (size_t cs = 0; cs < cellSpec.size(); cs++)
		{
			rc = cellSpec[cs].m_rect;
			cellSpec[cs].m_stepsH = howManySteps(rc.left(), rc.right());
			cellSpec[cs].m_stepsV = howManySteps(rc.bottom(), rc.top());
			howManyTurnsInRectangle(rc, turnsH, turnsV);
			cellSpec[cs].m_procId = getCoverageProcType(cellSpec[cs].m_startCorner, cellSpec[cs].m_endCorner, turnsH, turnsV);
			ptx = getCoverageProcPathWithResolution(cellSpec[cs].m_procId, cellSpec[cs].m_stepsH, cellSpec[cs].m_stepsV);

			if ((cs > 0) && (ptx.size() > 0))
			{
				startPoint = path[path.size() - 1];
				endPoint = aPoint(ptx[0].x() + rc.left(), ptx[0].y() + rc.bottom());

				linkPath = pathPlan(startPoint, endPoint);
				addPathToPath(path, linkPath);
			}
			m_logfile << "OFFS  (" << rc.bottomLeft().x() << "," << rc.bottomLeft().y() << ")";
			addPathToPathWithOffset(path, ptx, rc.bottomLeft());
		}
	}

	double CoveragePlanner::optimizeCellPath(CoveragePlannerCell &cell)
	{
		int procId, divider = 10;
		double step;
		if (cell.m_rect.width() > cell.m_rect.height())	//	Sviluppo in verticale per linee orizzontali
		{
			switch (m_direction)
			{
			case NE_DIRECTION: procId = RU_PROC; break;
			case SE_DIRECTION: procId = RD_PROC; break;
			case NW_DIRECTION: procId = LU_PROC; break;
			case SW_DIRECTION: procId = LD_PROC; break;
			}
			step = cell.m_rect.height() / (double)divider;
		}
		else//	Sviluppo in orizzontale per linee verticali
		{
			switch (m_direction)
			{
			case NE_DIRECTION: procId = UR_PROC; break;
			case SE_DIRECTION: procId = DR_PROC; break;
			case NW_DIRECTION: procId = UL_PROC; break;
			case SW_DIRECTION: procId = DL_PROC; break;
			}
			step = cell.m_rect.width() / (double)divider;
		}
		cell.m_procId = procId;
		return step;
	}

	void CoveragePlanner::calculatePathWithoutResolution(coverageCellVec cellSpec, pointVec &path)
	{
		pointVec linkPath;
		aPoint startPoint;
		aPoint endPoint;
		aRectangle rc;
		pointVec ptx;
		double step;

		m_logfile << "START (" << m_pointStart.x() << "," << m_pointStart.y() << ")" << endl;
		m_logfile << "END   (" << m_pointEnd.x() << "," << m_pointEnd.y() << ")" << endl;

		for (size_t cs = 0; cs < cellSpec.size(); cs++)
		{
			step = optimizeCellPath(cellSpec[cs]);
			rc = cellSpec[cs].m_rect;
			ptx = getCoverageProcPathWithoutResolution(cellSpec[cs].m_procId, rc, step);

			if ((cs > 0) && (ptx.size() > 0))
			{
				startPoint = path[path.size() - 1];
				endPoint = aPoint(ptx[0].x(), ptx[0].y());

				linkPath = pathPlan(startPoint, endPoint);
				addPathToPath(path, linkPath);
			}
			addPathToPath(path, ptx);
		}
	}

}