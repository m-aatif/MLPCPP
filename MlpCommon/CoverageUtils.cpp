#include "CoveragePlanner.h"

using namespace std;

namespace afarcloud
{
	/**
		UTILITIES
	*/
	void CoveragePlanner::logTotalCellsTurns(rectangleVec cells)
	{
		int turnsH, totalTurnsH;
		int turnsV, totalTurnsV;
		turnsH = totalTurnsH = 0;
		turnsV = totalTurnsV = 0;
		for (size_t i = 0; i < cells.size(); i++)
		{
			howManyTurnsInRectangle(cells[i], turnsH, turnsV);
			totalTurnsH += turnsH;
			totalTurnsV += turnsV;
		}
		m_logfile << "COMPOUND TOTAL TURNS (H/V) (" << totalTurnsH << "/" << totalTurnsV << ")" << endl;
	}

	void CoveragePlanner::addPathToPath(pointVec &pathToUpdate, pointVec pathToAdd)
	{
		pointVec::iterator ip = pathToAdd.begin();
		aPoint pt;
		while (ip != pathToAdd.end())
		{
			pt = *ip;
			pathToUpdate.push_back(pt);
			ip++;
		}
	}

	void CoveragePlanner::addPathToPathWithOffset(pointVec &pathToUpdate, pointVec pathToAdd, aPoint ptOffset)
	{
		pointVec::iterator ip = pathToAdd.begin();
		aPoint pt, ptn;
		while (ip != pathToAdd.end())
		{
			pt = *ip;
			ptn = aPoint(pt.x() + ptOffset.x(), pt.y() + ptOffset.y());
			pathToUpdate.push_back(ptn);
			ip++;
		}
	}

}
