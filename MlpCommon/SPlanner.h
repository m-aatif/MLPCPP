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
	typedef map<int32_t, aRectangle> rectangleMap;
	typedef std::vector<size_t> tagVec;

	class SNode
	{
	public:
		SNode() { m_origin = aPoint(0, 0); m_cost = 0.0; m_parentIndex = -1; }
		SNode(aPoint origin, double cost, int parentIndex) {
			m_origin = origin; m_cost = cost; m_parentIndex = parentIndex;
		}
		inline double cost() const { return m_cost; }
		inline int32_t parentIndex() const { return m_parentIndex; }
		inline aPoint origin() const { return m_origin; }
		inline double x() const { return m_origin.x(); }
		inline double y() const { return m_origin.y(); }

		inline void setCost(double cost) { m_cost = cost; }
		inline void setParentIndex(int32_t index) { m_parentIndex = index; }
	private:
		double m_cost;
		aPoint m_origin;
		int m_parentIndex;
	};

	typedef map<int32_t, SNode> snodeMap;

	class SPlanner : public Planner
	{
	public:
		SPlanner();
		SPlanner(aPoint ptStart, aPoint ptEnd, rectangleVec obstacles, aRectangle rectField, double resolution);
		void init(aPoint ptStart, aPoint ptEnd, rectangleVec obstacles, aRectangle rectField, double resolution);
		~SPlanner();
		pointVec plan();
		static int m_logNumber;

	private:
		int32_t getOpenSetLowestCostIndex(SNode goalNode);
		int32_t calcGridIndex(SNode node);
		double calcHeuristic(SNode node1, SNode node2);
		bool verifyNode(SNode node, SNode current);
		bool collides(SNode baseNode, SNode nearNode);
		bool collides(aPoint p1, aPoint p2);
		pointVec calcFinalPath(SNode goalNode);
		void logClosedSet(SNode goalNode);
		pointVec prunePath(pointVec path);

		aPoint m_pointStart, m_pointGoal;
		rectangleVec m_obstacles;	/*Forbidden areas as they are.*/
		aRectangle m_rectField;
		double m_resolution;
		int m_bSetUp;

		snodeMap m_openSet, m_closedSet;

		pointVec m_obsVertices;

		double m_min_x;
		double m_min_y;
		double m_max_x;
		double m_max_y;
		double m_width;
		double m_height;
	};
}

