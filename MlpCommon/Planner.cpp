#pragma once

#include <stdint.h>

#include "Planner.h"

using namespace std;
using namespace geon;

namespace afarcloud
{

	/**
	void	Planner::logPath(std::ofstream &_logfile, pointVec path)
	Summary: It writes the <path> vector into a file stream <_logfile>.
	Params: _logfile	ofstream		The stream to write to.
			path		pointVec		The vector of points to log.
	Return: none
	*/
	void	Planner::logPath(std::ofstream &_logfile, pointVec path)
	{
		_logfile << "PATH. " << path.size() << " WAYPOINTS" << endl;
		for (size_t p = 0; p < path.size(); p++)
			_logfile << "(" << path[p].x() << "," << path[p].y() << ")";
		_logfile << endl;
	}

	/**
	pointVec CoveragePlanner::prunePath(pointVec path)
	Summary: Prunes the <path> deleting the points belonging to the same line.
	Params: path			pointVec		The path to prune.
	Return:					pointVec		The pruned path.
	*/
	pointVec Planner::prunePath(pointVec path)
	{
		aPoint pt0, pt1, pt2;
		bool bBelongs;
		pointVec outPath;

		pt0 = path[0];
		outPath.push_back(pt0);
		pt1 = path[1];
		for (size_t n = 2; n < path.size(); n++)
		{
			pt2 = path[n];
			if (
				(pt0.x() == pt1.x()) && 
				(pt0.y() == pt1.y())
				)
			{
				pt0 = pt1;
				pt1 = pt2;
			}
			else
			{
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
				}
			}
		}
		outPath.push_back(pt2);

		return outPath;
	}

}