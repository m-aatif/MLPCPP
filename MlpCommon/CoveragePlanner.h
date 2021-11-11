#pragma once

#include <stdint.h>

#include "Planner.h"
#include <boost/numeric/ublas/matrix.hpp>

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <fstream>

using namespace std;
using namespace geon;

namespace afarcloud
{
	class CoveragePlannerCell;
	class BoustrophedonCell;
	class BoustrophedonMotion;

	typedef map<int32_t, aRectangle> rectangleMap;
	typedef std::vector<size_t> tagVec;
	typedef std::vector<tagVec> tagSegmentVec;
	typedef std::vector<CoveragePlannerCell> coverageCellVec;
	/**
	cellStatusMap is a map whose key is a number that identifies the cell and whose value is the status of the cell.
	The key is a 32-bits unsigned integer number, and is composed as follows:
	RRRRCCCC
	The most significant 16-bits are the value of the cell row inside the grid of all cells.
	The least significant 16-bits are the value of the cell column inside the grid of all cells.
	Cell status are:
		CELL_STATUS_FORBIDDEN		A cell included in an obstacle.
		CELL_STATUS_EXTERNAL		A cell external to the area of interest.
		CELL_STATUS_UNEXPLORED		An unexplored cell.
		CELL_STATUS_EXPLORED		An explored cell.
	*/
	typedef	std::map<size_t, int> cellStatusMap;				///	tag->status map (Explored, Unexplored or Forbidden cell)

	typedef std::vector<BoustrophedonCell> boustrophedonCellVec;
	typedef std::vector<BoustrophedonMotion> boustrophedonMotionVec;

	/**
	BoustrophedonCell: A class that contains the boustrophedon cell parameters used by coverage planner algorithm.
	*/
	class BoustrophedonCell
	{
	public:
		BoustrophedonCell(int tag, bool bVisited = true)
		{
			m_bVisited = bVisited;
			m_tag = tag;
			m_neighborCells.clear();
		}

		int m_tag;
		bool m_bVisited;
		boustrophedonCellVec m_neighborCells;
	};

	/**
	CoveragePlannerCell: A class that contains the elementary cell parameters used by coverage planner algorithm.
	*/
	class CoveragePlannerCell
	{
	public:
		CoveragePlannerCell() { ; }
		CoveragePlannerCell(aRectangle rect, int startCorner, int endCorner) { m_rect = rect; m_startCorner = startCorner; m_endCorner = endCorner; }
		CoveragePlannerCell(aRectangle rect) { m_rect = rect; m_startCorner = 0; m_endCorner = 0; }

		aRectangle m_rect;
		int m_stepsH;
		int m_stepsV;
		int m_mask;
		int m_startCorner;
		int m_endCorner;
		int m_procId;
	};

	class BoustrophedonMotion
	{
	public:
		BoustrophedonMotion(int x, int y)
		{
			m_x = x;
			m_y = y;
		}

		inline int x() const { return m_x; }
		inline int y() const { return m_y; }
	private:
		int m_x;
		int m_y;
	};

	/**
	CoveragePlanner: A class that implements the coverage planner algorithm.
	*/
	class CoveragePlanner : public Planner
	{
	public:
		CoveragePlanner();
		CoveragePlanner(aPoint ptStart, aPoint ptEnd, rectangleVec obstacles, aRectangle rectField, aPoint ptHome, double resolution);
		void setUp(aPoint ptStart, aPoint ptEnd, rectangleVec obstacles, aRectangle rectField, aPoint ptHome, double resolution);
		~CoveragePlanner();
		pointVec plan();

		void setConnectionAlgorithm(int algorithm, double connAlgoRes, double connAlgoExtra1 = 0.0, double connAlgoExtra2 = 0.0 )
		{
			m_connectionAlgorithm = algorithm;
			m_connectionAlgorithmResolution = connAlgoRes;
			m_connectionAlgorithmExtraParam1 = connAlgoExtra1;
			m_connectionAlgorithmExtraParam2 = connAlgoExtra2;
		}

		void setDecompositionType(int decompositionType)
		{
			m_decompositionType = decompositionType;
		}

		rectangleVec getBasicCells() const { return m_basicCells; }
		rectangleVec getCompoundCells() const { return m_compoundCells; }
		rectangleVec getCompoundCellsH() const { return m_compoundCellsH; }
		rectangleVec getCompoundCellsV() const { return m_compoundCellsV; }

	private:
		inline size_t makeTag(int r, int c) { return (r << 16) + c; }
		inline int rowFromTag(size_t tag) { return (tag >> 16) & 0xFFFF; }
		inline int colFromTag(size_t tag) { return (tag & 0xFFFF); }
		enum
		{
			NONE_DIRECTION = -1,
			NE_DIRECTION = 0,
			SE_DIRECTION,
			NW_DIRECTION,
			SW_DIRECTION,
			N_DIRECTION,
			E_DIRECTION,
			S_DIRECTION,
			W_DIRECTION,
			//------------------------------
			HORZ_EXPANSION = 0,
			VERT_EXPANSION,
			//------------------------------
			NOT_ALIGNED = 0,
			TOP_ALIGNED,
			RIGHT_ALIGNED,
			BOTTOM_ALIGNED,
			LEFT_ALIGNED,
			//------------------------------
			NO_SIDE = 0,
			TOP_SIDE,
			RIGHT_SIDE,
			BOTTOM_SIDE,
			LEFT_SIDE,
			//------------------------------
			DISJOINT = 0,
			JOINT,
			JOINT_TL_BR,
			JOINT_T_L,
			JOINT_TC,
			JOINT_TR,
			//------------------------------
			UNSET_CORNER = 0,
			BL_CORNER = 1,
			TL_CORNER = 2,
			TR_CORNER = 4,
			BR_CORNER = 8,
			//------------------------------
			CELL_STATUS_FORBIDDEN = -2,
			CELL_STATUS_EXTERNAL,
			CELL_STATUS_UNEXPLORED,
			CELL_STATUS_EXPLORED,
			//------------------------------
			RU_PROC = 0,
			UR_PROC,
			RD_PROC,
			DR_PROC,
			LU_PROC,
			UL_PROC,
			LD_PROC,
			DL_PROC
		};
		aPoint m_pointStart, m_pointEnd, m_pointHome;	/** The points of interest */
		rectangleVec m_obstacles;						/** Forbidden areas as they are.*/
		aRectangle m_rectField;
		double m_resolution;
		int m_direction;
		int m_expansion;
		int m_bSetUp;
		int m_connectionAlgorithm;
		double m_connectionAlgorithmResolution;
		double m_connectionAlgorithmExtraParam1;
		double m_connectionAlgorithmExtraParam2;
		int m_decompositionType;

		doubleVec m_xCoords;
		doubleVec m_yCoords;
		rectangleVec m_forbiddenRects;	/*Forbidden areas resolution fitted, cut to fit coverage area.*/
		pointVec m_vertices;
		rectangleVec m_basicCells;
		rectangleVec m_compoundCells;
		rectangleVec m_compoundCellsH;
		rectangleVec m_compoundCellsV;
		rectangleMap m_basicCellRectangleMap;
		cellStatusMap m_basicCellStatusMap;

		void calcCoordinatesWithResolution();
		void calcCoordinatesWithoutResolution();
		void createBasicCellStatusMap();
		int howManySteps(double from, double to);
		void howManyTurnsInRectangle(aRectangle aRect, int &turnsH, int &turnsV);
		void sortAscendingCoordinates(doubleVec &coord);
		void sortDescendingCoordinates(doubleVec &coord);
		double getNearestInGridFromLesser(double v0, double v1, bool bConsiderTouch = true);
		double getNearestInGridFromGreater(double v0, double v1, bool bConsiderTouch = true);
		bool isPointInsideAnObstacle(aPoint pt);
		bool intersectsAnObstacle(aRectangle ar);
		bool intersectsAnObstacle(aPoint pt1, aPoint pt2);
		aRectangle getObstacleIntercepted(aPoint pt1, aPoint pt2);
		pointVec prunePath(pointVec path, int &turns);

		int getCoverageProcType(int startCorner, int endCorner, int stepsHorz, int stepsVert);
		pointVec getCoverageProcPathWithResolution(int procType, int stepsHorz, int stepsVert);
		pointVec getCoverageProcPathWithoutResolution(int procType, aRectangle rc, double step);

		pointVec getRegularPathRightUp(int stepsHorz, int stepsVert);
		pointVec getRegularPathRightUpWithoutResolution(aRectangle rc, double step);
		pointVec getRegularPathUpRight(int stepsHorz, int stepsVert);
		pointVec getRegularPathUpRightWithoutResolution(aRectangle rc, double step);

		pointVec getRegularPathRightDown(int stepsHorz, int stepsVert);
		pointVec getRegularPathRightDownWithoutResolution(aRectangle rc, double step);
		pointVec getRegularPathDownRight(int stepsHorz, int stepsVert);
		pointVec getRegularPathDownRightWithoutResolution(aRectangle rc, double step);

		pointVec getRegularPathLeftUp(int stepsHorz, int stepsVert);
		pointVec getRegularPathLeftUpWithoutResolution(aRectangle rc, double step);
		pointVec getRegularPathUpLeft(int stepsHorz, int stepsVert);
		pointVec getRegularPathUpLeftWithoutResolution(aRectangle rc, double step);

		pointVec getRegularPathLeftDown(int stepsHorz, int stepsVert);
		pointVec getRegularPathLeftDownWithoutResolution(aRectangle rc, double step);
		pointVec getRegularPathDownLeft(int stepsHorz, int stepsVert);
		pointVec getRegularPathDownLeftWithoutResolution(aRectangle rc, double step);

		/*	CoverageUtils.cpp	*/
		void addPathToPath(pointVec &pathToUpdate, pointVec pathToAdd);
		void addPathToPathWithOffset(pointVec &pathToUpdate, pointVec pathToAdd, aPoint ptOffset);
		void logTotalCellsTurns(rectangleVec cells);


		void logBoustrophedonCellList(boustrophedonCellVec cellList);
		int getBestDirectionForAggregation(size_t startCellIndex, boustrophedonCellVec cellList);

		/*	Strategies.cpp	*/
		pointVec strategyAggregation();
		pointVec strategyBoustrophedon();
		pointVec pathPlan(aPoint ptStart, aPoint ptEnd);
		size_t findClosestCellToStartPosition(pointVec &path, coverageCellVec &cells);
		void calculatePathWithResolution(coverageCellVec cellSpec, pointVec &path);
		void calculatePathWithoutResolution(coverageCellVec cellSpec, pointVec &path);
		void step1fwd(int tag, map<size_t, int> &cellStatusMap, boustrophedonCellVec &cellList, int lastMotionIndex);
		void setCellSpecCorners(coverageCellVec &cellSpec);
		double optimizeCellPath(CoveragePlannerCell &cell);

		/*	Aggregators.cpp	*/
		void findAndJoinRectanglesLeft(rectangleVec inRects, rectangleVec &outRects);
		void findAndJoinRectanglesRight(rectangleVec inRects, rectangleVec &outRects);
		void findAndJoinRectanglesBottom(rectangleMap inRectsMap, size_t xSize, size_t ySize, rectangleVec &outRects);
		void findAndJoinRectanglesTop(rectangleMap inRectsMap, size_t xSize, size_t ySize, rectangleVec &outRects);
		void adjustRectanglesLeft(rectangleVec &outRects);
		void adjustRectanglesRight(rectangleVec &outRects);
		void adjustRectanglesBottom(rectangleVec &outRects);
		void adjustRectanglesTop(rectangleVec &outRects);
		void aggregateRectanglesA();

		std::ofstream m_logfile;

		boustrophedonMotionVec m_motion;
		void setMotion();

		void setHelperStrings();
		stringVec m_alignmentStr;
		stringVec m_joinStatusStr;
		stringVec m_pointStr;
		stringVec m_procStr;
		stringVec m_expansionStr;
	};
}

