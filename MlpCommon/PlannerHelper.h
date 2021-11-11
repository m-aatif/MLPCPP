#pragma once

#include "PlannerService.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/json.hpp>
#include <iostream>
#include <iomanip>
#include "Geometry.h"
#include "Configurator.h"

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <fstream>

using namespace boost::numeric::ublas;
using namespace geon;

namespace json = boost::json;

#define HEX( x ) setw(2) << setfill('0') << hex << (int)(x)
#define PRINT_DOUBLE_HEX( d ) \
{ \
	double x = (d); \
	uint8_t *px = (uint8_t*)&x; \
	std::cout << HEX(px[0]) << HEX(px[1]) << HEX(px[2]) << HEX(px[3]) << HEX(px[4]) << HEX(px[5]) << HEX(px[6]) << HEX(px[7]) << endl; \
}

namespace afarcloud
{
#ifdef _WINDOWS
	typedef	struct _TaskStruct
	{
		int32_t id;
		TaskType::type taskType;
	} TaskStruct;
#endif // _WINDOWS

	class MapStuff {
	public:
		MapStuff()
		{
		}

		//----------------------------------------------
		double absMinLon;
		double absMinLat;
		double absMaxLon;
		double absMaxLat;
		double absDeltaLon;
		double absDeltaLat;
		double n_distance;
		//----------------------------------------------
		/**
			sortedTasks
		*/
		std::vector<Task> sortedTasks;

		aRectangle navArea;

		aRectangle randArea;
		aPoint start;
		aPoint goal;
		std::vector<aRectangle> obstacles_rect;
		double obstacleMinimumSize;

		aRectangle randAreaN;
		aPoint startN;
		aPoint goalN;
		std::vector<aRectangle> obstacles_rectN;
	};

	class PlannerHelper
	{
	public:
		PlannerHelper(
#ifdef _WINDOWS
			CWnd *pWnd
#endif // _WINDOWS
		);

		~PlannerHelper();

		void	changeMissionEndianness(Mission &context);
		void	dumpMission(const Mission &context, std::ostream& os);
		void	dumpMissionAsJson(const Mission &context, std::ostream& os);

		void	preparePlan(Mission &context);

		static const double DEG2RAD;
		static const double RAD2DEG;
		static const double EARTH_MEAN_RADIUS;
		static double	haversine(aPoint pt1, aPoint pt2);
		static double	normalize(double value, double minVal, double maxVal);
		static void s2ws(const std::string& s, wchar_t* buf);

#ifdef _WINDOWS
		static wchar_t* formatAndAllocateString(wchar_t* format, ...);
#endif // _WINDOWS
	private:
		std::string	enquote_string(std::string orig_string);
		void	reversePositionInPlace(Position &pos);
		void	reverseOrientationInPlace(Orientation &orientation);
		void	matrixRowLimits(matrix<double> mtx, int row, double &minVal, double &maxVal);

		aPoint	normalizePoint(aPoint ptSrc);
		double	denormalize(double normalizedValue, double minVal, double maxVal);
		void	coordNormalization(
			matrix<double> mxIn, matrix<double> myIn,
			matrix<double> &mxOut, matrix<double> &myOut,
			double xMin, double xMax, double yMin, double yMax);

		void	preprocessingForOptimalPath(Mission &context);
		void	sortTasks(Mission &context);
		void	preprocessingPathPlanning(Mission &context, uint32_t taskIndex, uint32_t vehicleIndex);
		void	preprocessingPositions(Mission &context, uint32_t taskIndex, uint32_t vehicleIndex);

		void	checkOnFirstTask(bool bProceed, Mission context, uint32_t vehicleIndex, pointVec &path);
		void	checkOnLastTask(bool bProceed, Mission &context, uint32_t vehicleIndex, pointVec &path);
		bool	checkPlan(Mission context);

		void	transit(Mission &context, uint32_t taskIndex, bool bLastTask);
		void	survey(Mission &context, uint32_t taskIndex, bool bLastTask);
		void	inspect(Mission &context, uint32_t taskIndex, bool bLastTask);

		void	fillCommands(Mission &context, std::vector<aPoint> path, uint32_t taskIndex, uint32_t vehicleIndex, bool bLastTask);
		void	fillCommandsForTransitFlyingVehicles(Mission &context, std::vector<aPoint> path, Task &task, uint32_t vehicleIndex, bool bLastTask);
		void	fillCommandsForTransitOtherVehicles(Mission &context, std::vector<aPoint> path, Task &task, uint32_t vehicleIndex, bool bLastTask);
		void	fillCommandsForSurvey(Mission &context, std::vector<aPoint> path, Task &task, uint32_t vehicleIndex, bool bLastTask);
		void	fillCommandsForInspect(Mission &context, std::vector<aPoint> path, Task &task, uint32_t vehicleIndex, bool bLastTask);

		Command	fillCommand(CommandType::type type, Task relatedTask, double startTime, double endTime, std::vector<double> params);
		inline aPoint	LlPositionToPoint(Position pos);
		uint32_t	findVehicleIndexById(Mission &context, uint32_t id);
		inline	bool	isAFlyingVehicle(VehicleType::type type);
		std::vector<aPoint> denormalizePath(std::vector<aPoint> npath);

		double  m_startTime, m_endTime;
		int32_t m_id;
		double	m_speed;
		std::vector<Command> m_cmdList;

		static	Position aPointToPosition(aPoint pt, double altitude);
		static	aPoint positionToAPoint(Position pos);

		MapStuff m_mapStuff;
		std::ofstream m_logfile;
		void	logPath(Task task, std::vector<aPoint> path);

		void	loadAlgorithmConfiguration();

		bool	getTaskAlgorithmDescr(std::string taskName, TaskAlgorithm &ta);
		int32_t	getTaskAlgorithm(TaskAlgorithm ta);

		algorithmMap m_algoMap;
		taskAlgorithmMap m_taskAlgoMap;

		double	altitudeMagic(Task task);

#ifdef _WINDOWS
		void	postRectangle(UINT msgId, WPARAM wParam, aRectangle aRect);
		void	postRegion(UINT msgId, WPARAM wParam, Region region);
		void	postPosition(UINT msgId, WPARAM wParam, Position position);
		void	postMissionInfo(Mission mission);
		void	postPath(int taskIndex, std::vector<aPoint> path);
#endif // _WINDOWS
	protected:
#ifdef _WINDOWS
		CWnd *m_pWnd;
#endif // _WINDOWS
	};

}	//	namespace