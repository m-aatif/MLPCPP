#pragma once

#include <stdint.h>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <fstream>

#include "Geometry.h"

using namespace std;
using namespace geon;

namespace afarcloud
{
	/**
	Some useful types definitions.
	*/
	typedef std::vector<double> doubleVec;
	typedef std::vector<string> stringVec;

	/**
	Planner: A class that implements the base class for every planner algorithm.
	This is an abstract class because of protected constructors. Each derived class must implement its own constructor.
	Moreover, each derived class must implement the planning function.
	*/
	class Planner
	{
	public:
		/**
		virtual pointVec plan() = 0: The planner function the derived classes must implement.
		*/
		virtual pointVec plan() = 0;
		virtual ~Planner() { ; }

		void enableLog(bool bEnable) { m_bLog = bEnable; }

		static	pointVec prunePath(pointVec path);
		static	void	logPath(std::ofstream &_logfile, pointVec path);
	protected:
		/**
		Planner(): The default Planner costructor.
		*/
		Planner()
		{
			;
		}
		/**
		Planner(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField)
		Summary: It construct a Planner object.
		Params: ptStart		aPoint			The starting point.
				ptGoal		aPoint			The goal point.
				obstacles	rectangleVec	The vector containing the obstacles.
				rectField	aRectangle		The field of allowed positions.
		Return: none
		*/
		Planner(aPoint ptStart, aPoint ptGoal, rectangleVec obstacles, aRectangle rectField)
		{
			m_bLog = false;
		}

		rectangleVec m_obstacles;
		aRectangle m_rectField;
		bool m_bLog;
	};

}