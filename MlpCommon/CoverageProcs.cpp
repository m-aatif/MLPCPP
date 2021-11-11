#include "CoveragePlanner.h"

using namespace std;

namespace afarcloud
{
	int CoveragePlanner::getCoverageProcType(int startCorner, int endCorner, int turnsH, int turnsV)
	{
		int compoundCorners = startCorner + (endCorner << 4);
		if (turnsH < turnsV)	//	Sviluppo in orizzontale per linee verticali
		{
			switch (compoundCorners)
			{
			case (BL_CORNER | (TL_CORNER << 4)):return UR_PROC;		//*
			case (BL_CORNER | (TR_CORNER << 4)):return UR_PROC;
			case (BL_CORNER | (BR_CORNER << 4)):return UR_PROC;
			case (TL_CORNER | (TR_CORNER << 4)):return DR_PROC;
			case (TL_CORNER | (BR_CORNER << 4)):return DR_PROC;
			case (TL_CORNER | (BL_CORNER << 4)):return DR_PROC;		//*
			case (TR_CORNER | (BR_CORNER << 4)):return DL_PROC;		//*
			case (TR_CORNER | (BL_CORNER << 4)):return DL_PROC;
			case (TR_CORNER | (TL_CORNER << 4)):return DL_PROC;
			case (BR_CORNER | (BL_CORNER << 4)):return UL_PROC;
			case (BR_CORNER | (TL_CORNER << 4)):return UL_PROC;
			case (BR_CORNER | (TR_CORNER << 4)):return UL_PROC;		//*
			}
		}
		else                    //	Sviluppo in verticale per linee orizzontali
		{
			switch (compoundCorners)
			{
			case (BL_CORNER | (TL_CORNER << 4)):return RU_PROC;
			case (BL_CORNER | (TR_CORNER << 4)):return RU_PROC;
			case (BL_CORNER | (BR_CORNER << 4)):return RU_PROC;		//*
			case (TL_CORNER | (TR_CORNER << 4)):return RD_PROC;
			case (TL_CORNER | (BR_CORNER << 4)):return RD_PROC;
			case (TL_CORNER | (BL_CORNER << 4)):return RD_PROC;		//*
			case (TR_CORNER | (BR_CORNER << 4)):return LD_PROC;
			case (TR_CORNER | (BL_CORNER << 4)):return LD_PROC;
			case (TR_CORNER | (TL_CORNER << 4)):return LD_PROC;		//*
			case (BR_CORNER | (BL_CORNER << 4)):return LU_PROC;		//*
			case (BR_CORNER | (TL_CORNER << 4)):return LU_PROC;
			case (BR_CORNER | (TR_CORNER << 4)):return LU_PROC;		//*
			}
		}
		return 0;
	}

	pointVec CoveragePlanner::getCoverageProcPathWithResolution(int procType, int stepsHorz, int stepsVert)
	{
		pointVec path;
		switch (procType)
		{
		case RU_PROC: path = getRegularPathRightUp(stepsHorz, stepsVert); break;
		case UR_PROC: path = getRegularPathUpRight(stepsHorz, stepsVert); break;
		case RD_PROC: path = getRegularPathRightDown(stepsHorz, stepsVert); break;
		case DR_PROC: path = getRegularPathDownRight(stepsHorz, stepsVert); break;
		case LU_PROC: path = getRegularPathLeftUp(stepsHorz, stepsVert); break;
		case UL_PROC: path = getRegularPathUpLeft(stepsHorz, stepsVert); break;
		case LD_PROC: path = getRegularPathLeftDown(stepsHorz, stepsVert); break;
		case DL_PROC: path = getRegularPathDownLeft(stepsHorz, stepsVert); break;
		}

		return path;
	}

	pointVec CoveragePlanner::getCoverageProcPathWithoutResolution(int procType, aRectangle rc, double step)
	{
		pointVec path;
		switch (procType)
		{
		default:
		case RU_PROC: path = getRegularPathRightUpWithoutResolution(rc, step); break;
		case UR_PROC: path = getRegularPathUpRightWithoutResolution(rc, step); break;
		case RD_PROC: path = getRegularPathRightDownWithoutResolution(rc, step); break;
		case DR_PROC: path = getRegularPathDownRightWithoutResolution(rc, step); break;
		case LU_PROC: path = getRegularPathLeftUpWithoutResolution(rc, step); break;
		case UL_PROC: path = getRegularPathUpLeftWithoutResolution(rc, step); break;
		case LD_PROC: path = getRegularPathLeftDownWithoutResolution(rc, step); break;
		case DL_PROC: path = getRegularPathDownLeftWithoutResolution(rc, step); break;
		}

		return path;
	}

	//	RIGHT/UP	---------------------------------------------------------------------------------------------

	pointVec CoveragePlanner::getRegularPathRightUp(int stepsHorz, int stepsVert)
	{
		double dx, dy, x, y;
		pointVec outPath;

		x = y = 0.0;
		dx = m_resolution;
		dy = m_resolution;
		for (int yy = 0; yy < stepsVert; yy++)
		{
			outPath.push_back(aPoint(x, y));
			x += dx * stepsHorz;
			outPath.push_back(aPoint(x, y));
			y += dy;
			dx = -dx;
		}

		return outPath;
	}

	pointVec CoveragePlanner::getRegularPathRightUpWithoutResolution(aRectangle rc, double step)
	{
		double dx, dy, x, y, epsilon = step / 10.0;
		pointVec outPath;

		x = rc.left(); y = rc.bottom();

		dx = rc.width();
		dy = step;
		while (y <= rc.top() + epsilon)
		{
			outPath.push_back(aPoint(x, y));
			x += dx;
			outPath.push_back(aPoint(x, y));

			y += dy;
			dx = -dx;
		}

		return outPath;
	}

	//	UP/RIGHT	---------------------------------------------------------------------------------------------

	pointVec CoveragePlanner::getRegularPathUpRight(int stepsHorz, int stepsVert)
	{
		double dx, dy, x, y;
		pointVec outPath;

		x = y = 0.0;
		dx = m_resolution;
		dy = m_resolution;

		for (int xx = 0; xx < stepsHorz; xx++)
		{
			outPath.push_back(aPoint(x, y));
			y += dy * stepsVert;
			outPath.push_back(aPoint(x, y));

			x += dx;
			dy = -dy;
		}

		return outPath;
	}

	pointVec CoveragePlanner::getRegularPathUpRightWithoutResolution(aRectangle rc, double step)
	{
		double dx, dy, x, y, epsilon = step / 10.0;
		pointVec outPath;

		x = rc.left();  y = rc.bottom();
		dx = step;
		dy = rc.height();

		while (x <= rc.right() + epsilon)
		{
			outPath.push_back(aPoint(x, y));
			y += dy;
			outPath.push_back(aPoint(x, y));

			x += dx;
			dy = -dy;
		}

		return outPath;
	}

	//	RIGHT/DOWN	---------------------------------------------------------------------------------------------

	pointVec CoveragePlanner::getRegularPathRightDown(int stepsHorz, int stepsVert)
	{
		double dx, dy, x, y;
		pointVec outPath;

		x = 0.0;
		y = (stepsVert - 1) * m_resolution;
		dx = m_resolution;
		dy = m_resolution;
		for (int yy = 0; yy < stepsVert; yy++)
		{
			outPath.push_back(aPoint(x, y));
			x += dx * stepsHorz;
			outPath.push_back(aPoint(x, y));

			y -= dy;
			dx = -dx;
		}

		return outPath;
	}

	pointVec CoveragePlanner::getRegularPathRightDownWithoutResolution(aRectangle rc, double step)
	{
		double dx, dy, x, y, epsilon = step / 10.0;
		pointVec outPath;

		x = rc.left();
		y = rc.top();
		dx = rc.width();
		dy = step;
		while (y >= rc.bottom() - epsilon)
		{
			outPath.push_back(aPoint(x, y));
			x += dx;
			outPath.push_back(aPoint(x, y));

			y -= dy;
			dx = -dx;
		}

		return outPath;
	}

	//	DOWN/RIGHT	---------------------------------------------------------------------------------------------

	pointVec CoveragePlanner::getRegularPathDownRight(int stepsHorz, int stepsVert)
	{
		double dx, dy, x, y;
		pointVec outPath;

		x = 0.0;
		y = (stepsVert - 1) * m_resolution;
		dx = m_resolution;
		dy = m_resolution;

		for (int xx = 0; xx < stepsHorz; xx++)
		{
			for (int yy = 0; yy < stepsVert; yy++)
			{
				outPath.push_back(aPoint(x, y));
				y -= dy;
			}
			y += dy;
			x += dx;
			dy = -dy;
		}

		return outPath;
	}

	pointVec CoveragePlanner::getRegularPathDownRightWithoutResolution(aRectangle rc, double step)
	{
		double dx, dy, x, y, epsilon = step / 10.0;
		pointVec outPath;

		x = rc.left();
		y = rc.top();
		dx = step;
		dy = rc.height();

		while (x <= rc.right() + epsilon)
		{
			outPath.push_back(aPoint(x, y));
			y -= dy;
			outPath.push_back(aPoint(x, y));

			x += dx;
			dy = -dy;
		}

		return outPath;
	}

	//---------------------------------------------------------------------------------------------

	pointVec CoveragePlanner::getRegularPathLeftUp(int stepsHorz, int stepsVert)
	{
		double dx, dy, x, y;
		pointVec outPath;

		x = (stepsHorz - 1) * m_resolution; 
		y = 0.0;
		dx = m_resolution;
		dy = m_resolution;
		for (int yy = 0; yy < stepsVert; yy++)
		{
			for (int xx = 0; xx < stepsHorz; xx++)
			{
				outPath.push_back(aPoint(x, y));
				x -= dx;
			}
			x += dx;
			y += dy;
			dx = -dx;
		}

		return outPath;
	}

	pointVec CoveragePlanner::getRegularPathLeftUpWithoutResolution(aRectangle rc, double step)
	{
		double dx, dy, x, y, epsilon = step / 10.0;
		pointVec outPath;

		x = rc.right();
		y = rc.bottom();
		dx = rc.width();
		dy = step;

		while (y <= rc.top() + epsilon)
		{
			outPath.push_back(aPoint(x, y));
			x -= dx;
			outPath.push_back(aPoint(x, y));

			y -= dy;
			dx = -dx;
		}

		return outPath;
	}

	//---------------------------------------------------------------------------------------------

	pointVec CoveragePlanner::getRegularPathUpLeft(int stepsHorz, int stepsVert)
	{
		double dx, dy, x, y;
		pointVec outPath;

		x = (stepsHorz - 1) * m_resolution;
		y = 0.0;
		dx = m_resolution;
		dy = m_resolution;

		for (int xx = 0; xx < stepsHorz; xx++)
		{
			for (int yy = 0; yy < stepsVert; yy++)
			{
				outPath.push_back(aPoint(x, y));
				y += dy;
			}
			y -= dy;
			x -= dx;
			dy = -dy;
		}

		return outPath;
	}

	pointVec CoveragePlanner::getRegularPathUpLeftWithoutResolution(aRectangle rc, double step)
	{
		double dx, dy, x, y, epsilon = step / 10.0;
		pointVec outPath;

		x = rc.right();
		y = rc.bottom();
		dx = rc.width();
		dy = step;

		while (y <= rc.top() + epsilon)
		{
			outPath.push_back(aPoint(x, y));
			x -= dx;
			outPath.push_back(aPoint(x, y));

			y += dy;
			dx = -dx;
		}

		return outPath;
	}

	//---------------------------------------------------------------------------------------------

	pointVec CoveragePlanner::getRegularPathLeftDown(int stepsHorz, int stepsVert)
	{
		double dx, dy, x, y;
		pointVec outPath;

		x = (stepsHorz - 1) * m_resolution;
		y = (stepsVert - 1) * m_resolution;
		dx = m_resolution;
		dy = m_resolution;
		for (int yy = 0; yy < stepsVert; yy++)
		{
			for (int xx = 0; xx < stepsHorz; xx++)
			{
				outPath.push_back(aPoint(x, y));
				x -= dx;
			}
			x += dx;
			y -= dy;
			dx = -dx;
		}

		return outPath;
	}

	pointVec CoveragePlanner::getRegularPathLeftDownWithoutResolution(aRectangle rc, double step)
	{
		double dx, dy, x, y, epsilon = step / 10.0;
		pointVec outPath;

		x = rc.right();
		y = rc.top();
		dx = rc.width();
		dy = step;

		while (y >= rc.bottom() - epsilon)
		{
			outPath.push_back(aPoint(x, y));
			x -= dx;
			outPath.push_back(aPoint(x, y));

			y -= dy;
			dx = -dx;
		}

		return outPath;
	}

	//---------------------------------------------------------------------------------------------

	pointVec CoveragePlanner::getRegularPathDownLeft(int stepsHorz, int stepsVert)
	{
		double dx, dy, x, y;
		pointVec outPath;

		x = (stepsHorz - 1) * m_resolution;
		y = (stepsVert - 1) * m_resolution;
		dx = m_resolution;
		dy = m_resolution;

		for (int xx = 0; xx < stepsHorz; xx++)
		{
			for (int yy = 0; yy < stepsVert; yy++)
			{
				outPath.push_back(aPoint(x, y));
				y -= dy;
			}
			y += dy;
			x -= dx;
			dy = -dy;
		}

		return outPath;
	}

	pointVec CoveragePlanner::getRegularPathDownLeftWithoutResolution(aRectangle rc, double step)
	{
		double dx, dy, x, y, epsilon = step / 10.0;
		pointVec outPath;

		x = rc.right();
		y = rc.top();
		dx = step;
		dy = rc.height();

		while (x >= rc.left() - epsilon)
		{
			outPath.push_back(aPoint(x, y));
			y -= dy;
			outPath.push_back(aPoint(x, y));

			x -= dx;
			dy = -dy;
		}

		return outPath;
	}

}
