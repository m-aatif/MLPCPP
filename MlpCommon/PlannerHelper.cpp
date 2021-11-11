#ifdef _WINDOWS
#include "pch.h"
#endif // _WINDOWS

#include "PlannerHelper.h"

#include <boost\endian.hpp>
#include "AStarPlanner.h"
#include "SPlanner.h"
#include "CoveragePlanner.h"

#ifdef _WINDOWS
#include "WinMessages.h"
#endif // _WINDOWS

using namespace std;
using namespace boost::endian;
using namespace boost::math::double_constants;

#define	COLON					": "
#define	COMMA					","
#define	OB						"("
#define	CB						")"
#define	OSB						"["
#define	CSB						"]"
#define	OCB						"{"
#define	CCB						"}"

namespace afarcloud
{
	const double PlannerHelper::DEG2RAD = pi / 180.0;
	const double PlannerHelper::RAD2DEG = 180.0 / pi;
	const double PlannerHelper::EARTH_MEAN_RADIUS = 6371000.0;

	PlannerHelper::PlannerHelper(
#ifdef _WINDOWS
		CWnd *pWnd
#endif // _WINDOWS
	)
	{
#ifdef _WINDOWS
		m_pWnd = pWnd;
#endif // _WINDOWS
		m_startTime = 0;
		m_id = 0;

		m_logfile = std::ofstream("./mission.log");
	}

	PlannerHelper::~PlannerHelper()
	{
		m_logfile.close();
	}

	void PlannerHelper::reversePositionInPlace(Position &pos)
	{
		endian_reverse_inplace(pos.longitude);
		endian_reverse_inplace(pos.latitude);
		endian_reverse_inplace(pos.altitude);
	}

	void PlannerHelper::s2ws(const std::string& s, wchar_t* buf)
	{
		int len;
		int slength = (int)s.length() + 1;
		len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
		MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
	}

#ifdef _WINDOWS
	wchar_t* PlannerHelper::formatAndAllocateString(wchar_t* format, ...)
	{
		va_list args;
		int     len;
		wchar_t    *buffer;

		// retrieve the variable arguments
		va_start(args, format);

		len = _vscwprintf(format, args) // _vscprintf doesn't count
			+ 1; // terminating '\0'

		buffer = (wchar_t*)malloc(len * sizeof(wchar_t));
		if (0 != buffer)
		{
			vswprintf(buffer, len, format, args);
		}
		va_end(args);

		return buffer;
	}

#endif // _WINDOWS

	void PlannerHelper::reverseOrientationInPlace(Orientation &orientation)
	{
		endian_reverse_inplace(orientation.roll);
		endian_reverse_inplace(orientation.pitch);
		endian_reverse_inplace(orientation.yaw);
	}

	void PlannerHelper::changeMissionEndianness(Mission &context)
	{
		for (uint32_t np = 0; np < context.navigationArea.area.size(); np++)
		{
			reversePositionInPlace(context.navigationArea.area.at(np));
		}
		for (uint32_t na = 0; na < context.forbiddenArea.size(); na++)
		{
			for (uint32_t np = 0; np < context.forbiddenArea.at(na).area.size(); np++)
			{
				reversePositionInPlace(context.forbiddenArea.at(na).area.at(np));
			}
		}
		for (uint32_t np = 0; np < context.homeLocation.size(); np++)
		{
			reversePositionInPlace(context.homeLocation.at(np));
		}
		for (uint32_t nv = 0; nv < context.vehicles.size(); nv++)
		{
			endian_reverse_inplace(context.vehicles.at(nv).maxSpeed);
			endian_reverse_inplace(context.vehicles.at(nv).safetyDistance);
			reversePositionInPlace(context.vehicles.at(nv).stateVector.position);
			endian_reverse_inplace(context.vehicles.at(nv).stateVector.lastUpdate);
			reverseOrientationInPlace(context.vehicles.at(nv).stateVector.orientation);
			endian_reverse_inplace(context.vehicles.at(nv).stateVector.gimbalPitch);
			endian_reverse_inplace(context.vehicles.at(nv).stateVector.linearSpeed);
		}
		for (uint32_t nt = 0; nt < context.tasks.size(); nt++)
		{
			endian_reverse_inplace(context.tasks.at(nt).speed);
			endian_reverse_inplace(context.tasks.at(nt).altitude);
			endian_reverse_inplace(context.tasks.at(nt).range);
			endian_reverse_inplace(context.tasks.at(nt).startTime);
			endian_reverse_inplace(context.tasks.at(nt).endTime);
			for (uint32_t np = 0; np < context.tasks.at(nt).area.area.size(); np++)
			{
				reversePositionInPlace(context.tasks.at(nt).area.area.at(np));
			}
			reverseOrientationInPlace(context.tasks.at(nt).bearing);
		}
		for (uint32_t nc = 0; nc < context.commands.size(); nc++)
		{
			for (uint32_t p = 0; p < context.commands[nc].params.size(); p++)
			{
				endian_reverse_inplace(context.commands[nc].params[p]);
			}
		}
	}

	void PlannerHelper::dumpMission(const Mission &context, std::ostream& os)
	{
		Vehicle v;
		Position p;
		Region r;
		Task t;
		Orientation o;
		Equipment e;
		Command c;

		std::string indent4 = "";
		std::string indent8 = "";
		std::string indent12 = "";
		std::string indent16 = "";
		indent4.append(4, ' ');
		indent8.append(8, ' ');
		indent12.append(12, ' ');
		indent16.append(16, ' ');

		os << "Mission Id     :" << context.missionId << endl;
		os << "Name           :" << context.name << endl;
		os << "Navigation Area:" << endl;
		for (uint32_t np = 0; np < context.navigationArea.area.size(); np++)
		{
			p = context.navigationArea.area.at(np);
			os << indent4 << "LLH (" << p.longitude << "," << p.latitude << "," << p.altitude << ")" << endl;;
		}
		os << "Forbidden Area :" << endl;
		for (uint32_t na = 0; na < context.forbiddenArea.size(); na++)
		{
			os << indent4 << "Area:" << na << endl;
			r = context.forbiddenArea.at(na);
			for (uint32_t np = 0; np < r.area.size(); np++)
			{
				p = r.area.at(np);
				os << indent8 << "LLH (" << p.longitude << "," << p.latitude << "," << p.altitude << ")" << endl;
			}
		}
		os << "Home Location  :" << endl;
		for (uint32_t np = 0; np < context.homeLocation.size(); np++)
		{
			p = context.homeLocation.at(np);
			os << indent4 << "LLH (" << p.longitude << "," << p.latitude << "," << p.altitude << ")" << endl;
		}
		os << "Vehicles       :" << endl;
		for (uint32_t nv = 0; nv < context.vehicles.size(); nv++)
		{
			os << indent4 << "Vehicle:" << nv << endl;
			v = context.vehicles.at(nv);
			os << indent8 << "Name           :" << v.name << endl;
			os << indent8 << "Id             :" << v.id << endl;
			os << indent8 << "Type           :" << _VehicleType_VALUES_TO_NAMES.at(v.type) << endl;
			//os << indent8 << "Type           :" << v.type << endl;
			os << indent8 << "Max Speed      :" << v.maxSpeed << endl;
			os << indent8 << "Running Time   :" << v.maxRunningTime << endl;
			os << indent8 << "Equipments:" << endl;
			for (uint32_t ne = 0; ne < v.equipments.size(); ne++)
			{
				os << indent12 << "Equipment:" << ne << endl;
				e = v.equipments.at(ne);
				os << indent16 << "ID             :" << e.id << endl;
				os << indent16 << "ISO ID         :" << e.isoId << endl;
				os << indent16 << "Type           :" << _EquipmentType_VALUES_TO_NAMES.at(e.type) << endl;
				os << indent16 << "Name           :" << e.name << endl;
			}
			//	Equipments
			//	Capabilities
			os << indent8 << "State vector:" << endl;
			os << indent12 << "Gimbal pitch   :" << v.stateVector.gimbalPitch << endl;
			os << indent12 << "Last update    :" << v.stateVector.lastUpdate << endl;
			os << indent12 << "Linear speed   :" << v.stateVector.linearSpeed << endl;
			p = v.stateVector.position;
			os << indent12 << "Position       :LLH (" << p.longitude << "," << p.latitude << "," << p.altitude << ")" << endl;
			o = v.stateVector.orientation;
			os << indent12 << "Orientation    :RPY (" << o.roll << "," << o.pitch << "," << o.yaw << ")" << endl;
			os << indent8 << "Safety Distance:" << v.safetyDistance << endl;
		}
		os << "Tasks          :" << endl;
		for (uint32_t nt = 0; nt < context.tasks.size(); nt++)
		{
			t = context.tasks.at(nt);
			os << indent4 << "Task:" << nt << endl;
			os << indent8 << "Id             :" << t.id << endl;
			os << indent8 << "Mission Id     :" << t.missionId << endl;
			//os << indent8 << "Type           :" << t.taskTemplate.taskType << endl;
			os << indent8 << "Type           :" << _TaskType_VALUES_TO_NAMES.at(t.taskTemplate.taskType) << endl;
			os << indent8 << "Time Lapse     :" << t.timeLapse << endl;
			os << indent8 << "Start Time     :" << t.startTime << endl;
			os << indent8 << "End Time       :" << t.endTime << endl;
			os << indent8 << "Assigned V. Id :" << t.assignedVehicleId << endl;
			os << indent8 << "Parent Task Id :" << t.parentTaskId << endl;
			os << indent8 << "Repeat Count   :" << t.repeatCount << endl;
			os << indent8 << "Area           :" << endl;
			for (uint32_t np = 0; np < t.area.area.size(); np++)
			{
				p = t.area.area.at(np);
				os << indent12 << "LLH (" << p.longitude << "," << p.latitude << "," << p.altitude << ")" << endl;
			}
			os << indent8 << "Speed          :" << t.speed << endl;
			os << indent8 << "Altitude       :" << t.altitude << endl;
			os << indent8 << "Range          :" << t.range << endl;
			os << indent8 << "Time Lapse     :" << t.timeLapse << endl;
			o = t.bearing;
			os << indent8 << "Orientation    :RPY (" << o.roll << "," << o.pitch << "," << o.yaw << ")" << endl;
			os << indent8 << "Status         :" << _TaskCommandStatus_VALUES_TO_NAMES.at(t.taskStatus) << endl;
		}

		os << "Commands       :" << endl;
		for (uint32_t nc = 0; nc < context.commands.size(); nc++)
		{
			c = context.commands.at(nc);
			t = c.relatedTask;
			os << indent4 << "Command:" << nc << endl;
			os << indent8 << "Id             :" << c.id << endl;
			os << indent8 << "Related Task ID:" << t.id << " (" << _TaskType_VALUES_TO_NAMES.at(t.taskTemplate.taskType) << ")" << endl;
			os << indent8 << "Type           :" << _CommandType_VALUES_TO_NAMES.at(c.commandType) << endl;
			os << indent8 << "Status         :" << _TaskCommandStatus_VALUES_TO_NAMES.at(c.commandStatus) << endl;
			os << indent8 << "Start Time     :" << c.startTime << endl;
			os << indent8 << "End Time       :" << c.endTime << endl;
			os << indent8 << "Params         :";
			for (uint32_t np = 0; np < c.params.size(); np++)
				os << " " << c.params.at(np);
			os << endl;
		}
	}

	std::string	PlannerHelper::enquote_string(std::string orig_string)
	{
		return "\"" + orig_string + "\"";
	}

	void PlannerHelper::dumpMissionAsJson(const Mission &context, std::ostream& os)
	{
		Vehicle v;
		Position p;
		Region r;
		Task t;
		Command c;
		std::string indent4 = "";
		std::string indent8 = "";
		std::string indent12 = "";
		std::string indent16 = "";
		std::string indent20 = "";
		indent4.append(4, ' ');
		indent8.append(8, ' ');
		indent12.append(12, ' ');
		indent16.append(16, ' ');
		indent20.append(20, ' ');

		os << OCB << endl;
		os << indent4 << enquote_string("missionId") << COLON << context.missionId << COMMA << endl;
		os << indent4 << enquote_string("name") << COLON << enquote_string(context.name) << COMMA << endl;

		os << indent4 << enquote_string("navigationArea") << COLON << OSB << endl;
		for (uint32_t np=0; np<context.navigationArea.area.size(); np++)
		{
			p = context.navigationArea.area.at(np);
			os << indent8 << OCB << endl;
			os << indent12 << enquote_string("longitude") << COLON << p.longitude << COMMA;
			os << enquote_string("latitude") << COLON << p.latitude << COMMA;
			os << enquote_string("altitude") << COLON << p.altitude << endl;
			if (np < context.navigationArea.area.size() - 1)
				os << indent8 << CCB << COMMA << endl;
			else
				os << indent8 << CCB << endl;
		}
		os << indent4 << CSB << COMMA << endl;

		os << indent4 << enquote_string("forbiddenArea") << COLON << OSB << endl;
		for (uint32_t na = 0; na < context.forbiddenArea.size(); na++)
		{
			os << indent8 << OSB << endl;
			r = context.forbiddenArea.at(na);
			for (uint32_t np = 0; np < r.area.size(); np++)
			{
				p = r.area.at(np);
				os << indent12 << OCB << endl;
				os << indent16 << enquote_string("longitude") << COLON << p.longitude << COMMA;
				os << enquote_string("latitude") << COLON << p.latitude << COMMA;
				os << enquote_string("altitude") << COLON << p.altitude << endl;
				if (np < context.navigationArea.area.size() - 1)
					os << indent12 << CCB << COMMA << endl;
				else
					os << indent12 << CCB << endl;
			}
			if (na < context.forbiddenArea.size() - 1)
				os << indent8 << CSB << COMMA << endl;
			else
				os << indent8 << CSB << endl;
		}
		os << indent4 << CSB << COMMA << endl;

		os << indent4 << enquote_string("homeLocation") << COLON << OSB << endl;
		for (uint32_t np = 0; np < context.homeLocation.size(); np++)
		{
			p = context.homeLocation.at(np);
			os << indent8 << OCB << endl;
			os << indent12 << enquote_string("longitude") << COLON << p.longitude << COMMA;
			os << enquote_string("latitude") << COLON << p.latitude << COMMA;
			os << enquote_string("altitude") << COLON << p.altitude << endl;
			if (np < context.homeLocation.size() - 1)
				os << indent8 << CCB << COMMA << endl;
			else
				os << indent8 << CCB << endl;
		}
		os << indent4 << CSB << COMMA << endl;

		os << indent4 << enquote_string("vehicles") << COLON << OSB << endl;
		for (uint32_t nv = 0; nv < context.vehicles.size(); nv++)
		{
			v = context.vehicles.at(nv);
			os << indent8 << OCB << endl;
			os << indent12 << enquote_string("id") << COLON << v.id << COMMA << endl;
			os << indent12 << enquote_string("name") << COLON << enquote_string(v.name) << COMMA << endl;
			os << indent12 << enquote_string("type") << COLON << v.type << COMMA << endl;
			os << indent12 << enquote_string("maxSpeed") << COLON << v.maxSpeed << COMMA << endl;
			os << indent12 << enquote_string("maxRunningTime") << COLON << v.maxRunningTime << COMMA << endl;
			os << indent12 << enquote_string("safetyDistance") << COLON << v.safetyDistance << endl;
			if (nv < context.vehicles.size() - 1)
				os << indent8 << CCB << COMMA << endl;
			else
				os << indent8 << CCB << endl;
		}
		os << indent4 << CSB << COMMA << endl;

		os << indent4 << enquote_string("tasks") << COLON << OSB << endl;
		for (uint32_t nt = 0; nt < context.tasks.size(); nt++)
		{
			t = context.tasks.at(nt);
			os << indent8 << OCB << endl;
			os << indent12 << enquote_string("id") << COLON << t.id << COMMA << endl;
			os << indent12 << enquote_string("missionId") << COLON << t.missionId << COMMA << endl;
			os << indent12 << enquote_string("type") << COLON << t.taskTemplate.taskType << COMMA << endl;
			os << indent12 << enquote_string("timeLapse") << COLON << t.timeLapse << COMMA << endl;
			os << indent12 << enquote_string("startTime") << COLON << t.startTime << COMMA << endl;
			os << indent12 << enquote_string("endTime") << COLON << t.endTime << COMMA << endl;
			os << indent12 << enquote_string("assignedVehicleId") << COLON << t.assignedVehicleId << COMMA << endl;
			os << indent12 << enquote_string("parentTaskId") << COLON << t.parentTaskId << COMMA << endl;
			os << indent12 << enquote_string("repeatCount") << COLON << t.repeatCount << COMMA << endl;

			os << indent12 << enquote_string("area") << COLON << OSB << endl;
			for (uint32_t np = 0; np < t.area.area.size(); np++)
			{
				p = t.area.area.at(np);
				os << indent16 << OCB << endl;
				os << indent20 << enquote_string("longitude") << COLON << p.longitude << COMMA;
				os << enquote_string("latitude") << COLON << p.latitude << COMMA;
				os << enquote_string("altitude") << COLON << p.altitude << endl;
				if (np < t.area.area.size() - 1)
					os << indent16 << CCB << COMMA << endl;
				else
					os << indent16 << CCB << endl;
			}
			os << indent12 << CSB << COMMA << endl;

			os << indent12 << enquote_string("speed") << COLON << t.speed << COMMA << endl;
			os << indent12 << enquote_string("altitude") << COLON << t.altitude << COMMA << endl;
			os << indent12 << enquote_string("range") << COLON << t.range << COMMA << endl;
			os << indent12 << enquote_string("timeLapse") << COLON << t.timeLapse << COMMA << endl;
			os << indent12 << enquote_string("orientation") << COLON << OSB << endl;
			os << indent16 << OCB << endl;
			os << indent20 << enquote_string("roll") << COLON << t.bearing.roll << COMMA;
			os << enquote_string("pitch") << COLON << t.bearing.pitch << COMMA;
			os << enquote_string("yaw") << COLON << t.bearing.yaw << endl;
			os << indent16 << CCB << endl;
			os << indent12 << CSB << endl;
			if (nt < context.tasks.size() - 1)
				os << indent8 << CCB << COMMA << endl;
			else
				os << indent8 << CCB << endl;
		}
		os << indent4 << CSB << COMMA << endl;

		os << indent4 << enquote_string("commands") << COLON << OSB << endl;
		for (uint32_t nc = 0; nc < context.commands.size(); nc++)
		{
			c = context.commands.at(nc);
			os << indent8 << OCB << endl;
			os << indent12 << enquote_string("id") << COLON << c.id << COMMA << endl;
			os << indent12 << enquote_string("type") << COLON << c.commandType << COMMA << endl;
			os << indent12 << enquote_string("status") << COLON << c.commandStatus << COMMA << endl;
			os << indent12 << enquote_string("startTime") << COLON << c.startTime << COMMA << endl;
			os << indent12 << enquote_string("endTime") << COLON << c.endTime << COMMA << endl;
			os << indent12 << enquote_string("params") << COLON << OSB << endl;
			os << indent12 << CSB << endl;
			if (nc < context.commands.size() - 1)
				os << indent8 << CCB << COMMA << endl;
			else
				os << indent8 << endl;
		}
		os << indent4 << CSB << endl;

		os << CCB << endl;
	}

	void	PlannerHelper::loadAlgorithmConfiguration()
	{
		m_algoMap.clear();
		m_taskAlgoMap.clear();

		Configurator cfg("MLP.json");
		cfg.loadAlgorithmConfiguration(m_algoMap, m_taskAlgoMap);
	}

	bool	PlannerHelper::getTaskAlgorithmDescr(std::string taskName, TaskAlgorithm &ta)
	{
		try
		{
			ta = m_taskAlgoMap[taskName];
			return true;
		}
		catch (const std::exception&)
		{
			return false;
		}
	}

	int32_t	PlannerHelper::getTaskAlgorithm(TaskAlgorithm ta)
	{
		try
		{
			return m_algoMap[ta.algorithmName];
		}
		catch (const std::exception&)
		{
			return -1;
		}
	}

	aPoint	PlannerHelper::LlPositionToPoint(Position pos)
	{
		return aPoint(pos.longitude, pos.latitude);
	}

	uint32_t PlannerHelper::findVehicleIndexById(Mission &context, uint32_t id)
	{
		for (size_t v = 0; v < context.vehicles.size(); v++)
		{
			if (context.vehicles[v].id == id)
				return v;
		}
		return 0;
	}

	bool	PlannerHelper::isAFlyingVehicle(VehicleType::type type)
	{
		return (
			(type == VehicleType::UAV) ||
			(type == VehicleType::AUAV) ||
			(type == VehicleType::RUAV)
			);
	}

	/**
		Haversine formula
		pt1 & pt2 (lon,lat) in degrees
	*/
	double	PlannerHelper::haversine(aPoint pt1, aPoint pt2)
	{
		double deltaLonRads = (pt2.x() - pt1.x()) * DEG2RAD;
		double deltaLatRads = (pt2.y() - pt1.y()) * DEG2RAD;

		double s1 = sin(deltaLatRads / 2);
		double s2 = sin(deltaLonRads / 2);
		double a = s1 * s1 + cos(pt1.y()*DEG2RAD) * cos(pt2.y()*DEG2RAD) * s2*s2;
		return EARTH_MEAN_RADIUS * 2.0 * asin(sqrt(a));
	}

	double	PlannerHelper::normalize(double value, double minVal, double maxVal)
	{
		return (value - minVal) / (maxVal - minVal);
	}

	aPoint	PlannerHelper::normalizePoint(aPoint ptSrc)
	{
		double x = normalize(ptSrc.x(), m_mapStuff.absMinLon, m_mapStuff.absMaxLon);
		double y = normalize(ptSrc.y(), m_mapStuff.absMinLat, m_mapStuff.absMaxLat);
		return aPoint(x, y);
	}

	double	PlannerHelper::denormalize(double normalizedValue, double minVal, double maxVal)
	{
		return (normalizedValue * (maxVal - minVal) + minVal);
	}

	void	PlannerHelper::matrixRowLimits(matrix<double> mtx, int row, double &minVal, double &maxVal)
	{
		double vMin = 1E12, vMax = -1E12;

		for (size_t c = 0; c < mtx.size2(); c++)
		{
			if (mtx(row, c) < vMin)
				vMin = mtx(row, c);
			if (mtx(row, c) > vMax)
				vMax = mtx(row, c);
		}

		minVal = vMin;
		maxVal = vMax;
	}

	void	PlannerHelper::preprocessingForOptimalPath(Mission &context)
	{
		Position p;
		Region r;
		Vehicle v;
		Task t;

		/**
		*	Find scenario limits
		*/
		//----------------------------------------------
		p = context.navigationArea.area.at(0);
		m_mapStuff.absMinLon = m_mapStuff.absMaxLon = p.longitude;
		m_mapStuff.absMinLat = m_mapStuff.absMaxLat = p.latitude;
		for (uint32_t np = 1; np < context.navigationArea.area.size(); np++)
		{
			p = context.navigationArea.area.at(np);
			if (p.longitude < m_mapStuff.absMinLon)
				m_mapStuff.absMinLon = p.longitude;
			if (p.longitude > m_mapStuff.absMaxLon)
				m_mapStuff.absMaxLon = p.longitude;
			if (p.latitude < m_mapStuff.absMinLat)
				m_mapStuff.absMinLat = p.latitude;
			if (p.latitude > m_mapStuff.absMaxLat)
				m_mapStuff.absMaxLat = p.latitude;
		}
		Position p1 = context.navigationArea.area.at(0);
		Position p2 = context.navigationArea.area.at(2);
		m_mapStuff.navArea = aRectangle(
			aPoint(p1.longitude, p1.latitude),
			aPoint(p2.longitude, p2.latitude)
		);

		if (context.forbiddenArea.size() > 0)
		{
			for (uint32_t na = 0; na < context.forbiddenArea.size(); na++)
			{
				r = context.forbiddenArea.at(na);
				for (uint32_t np = 0; np < r.area.size(); np++)
				{
					p = r.area.at(np);
					if (p.longitude < m_mapStuff.absMinLon)
						m_mapStuff.absMinLon = p.longitude;
					if (p.longitude > m_mapStuff.absMaxLon)
						m_mapStuff.absMaxLon = p.longitude;
					if (p.latitude < m_mapStuff.absMinLat)
						m_mapStuff.absMinLat = p.latitude;
					if (p.latitude > m_mapStuff.absMaxLat)
						m_mapStuff.absMaxLat = p.latitude;
				}
			}
		}

		for (uint32_t nv = 0; nv < context.vehicles.size(); nv++)
		{
			v = context.vehicles.at(nv);
			p = v.stateVector.position;
			if (p.longitude < m_mapStuff.absMinLon)
				m_mapStuff.absMinLon = p.longitude;
			if (p.longitude > m_mapStuff.absMaxLon)
				m_mapStuff.absMaxLon = p.longitude;
			if (p.latitude < m_mapStuff.absMinLat)
				m_mapStuff.absMinLat = p.latitude;
			if (p.latitude > m_mapStuff.absMaxLat)
				m_mapStuff.absMaxLat = p.latitude;
		}

		for (uint32_t nt = 0; nt < context.tasks.size(); nt++)
		{
			t = context.tasks.at(nt);
			for (uint32_t np = 0; np < t.area.area.size(); np++)
			{
				p = context.navigationArea.area.at(np);
				if (p.longitude < m_mapStuff.absMinLon)
					m_mapStuff.absMinLon = p.longitude;
				if (p.longitude > m_mapStuff.absMaxLon)
					m_mapStuff.absMaxLon = p.longitude;
				if (p.latitude < m_mapStuff.absMinLat)
					m_mapStuff.absMinLat = p.latitude;
				if (p.latitude > m_mapStuff.absMaxLat)
					m_mapStuff.absMaxLat = p.latitude;
			}
		}
		m_mapStuff.absDeltaLon = m_mapStuff.absMaxLon - m_mapStuff.absMinLon;
		m_mapStuff.absDeltaLat = m_mapStuff.absMaxLat - m_mapStuff.absMinLat;
		//m_mapStuff.n_distance = hypot(m_mapStuff.absDeltaLon, m_mapStuff.absDeltaLat);
		m_mapStuff.n_distance = normalize(
			hypot(m_mapStuff.absDeltaLon, m_mapStuff.absDeltaLat),
			m_mapStuff.absMinLon,
			m_mapStuff.absMaxLon);

		m_mapStuff.obstacleMinimumSize = 0;
		if (context.forbiddenArea.size() > 0)
		{
			m_mapStuff.obstacleMinimumSize = 1.0E6;
			aRectangle rect;
			for (size_t na = 0; na < context.forbiddenArea.size(); na++)
			{
				r = context.forbiddenArea.at(na);
				Position p0 = r.area.at(0);
				Position p1 = r.area.at(2);
				rect = aRectangle(aPoint(p0.longitude, p0.latitude), aPoint(p1.longitude, p1.latitude));
				m_mapStuff.obstacles_rect.push_back(rect);

				if (rect.width() < m_mapStuff.obstacleMinimumSize)
					m_mapStuff.obstacleMinimumSize = rect.width();
				if (rect.height() < m_mapStuff.obstacleMinimumSize)
					m_mapStuff.obstacleMinimumSize = rect.height();

				rect = aRectangle(
					aPoint(
						normalize(p0.longitude, m_mapStuff.absMinLon, m_mapStuff.absMaxLon),
						normalize(p0.latitude, m_mapStuff.absMinLat, m_mapStuff.absMaxLat)
					),
					aPoint(
						normalize(p1.longitude, m_mapStuff.absMinLon, m_mapStuff.absMaxLon),
						normalize(p1.latitude, m_mapStuff.absMinLat, m_mapStuff.absMaxLat)
					)
				);
				m_mapStuff.obstacles_rectN.push_back(rect);
			}
		}

		m_mapStuff.randArea = aRectangle(
			m_mapStuff.absMinLon, m_mapStuff.absMinLat, m_mapStuff.absDeltaLon, m_mapStuff.absDeltaLat
		);

		m_mapStuff.randAreaN = aRectangle(
			normalize(m_mapStuff.absMinLon, m_mapStuff.absMinLon, m_mapStuff.absMaxLon),
			normalize(m_mapStuff.absMinLat, m_mapStuff.absMinLat, m_mapStuff.absMaxLat),
			1.0,
			1.0
		);

		//----------------------------------------------
	}

	void	PlannerHelper::coordNormalization(
		matrix<double> mxIn, matrix<double> myIn,
		matrix<double> &mxOut, matrix<double> &myOut,
		double xMin, double xMax, double yMin, double yMax)
	{
		double tmp;

		mxOut = matrix<double>(mxIn.size1(), mxIn.size2());
		myOut = matrix<double>(myIn.size1(), myIn.size2());

		for (unsigned i = 0; i < mxIn.size1(); ++i)
			for (unsigned j = 0; j < mxIn.size2(); ++j)
			{
				tmp = normalize(mxIn(i, j), xMin, xMax);
				mxOut(i, j) = tmp;
			}

		for (unsigned i = 0; i < myIn.size1(); ++i)
			for (unsigned j = 0; j < myIn.size2(); ++j)
			{
				tmp = normalize(myIn(i, j), yMin, yMax);
				myOut(i, j) = tmp;
			}
	}

	void	PlannerHelper::sortTasks(Mission &context)
	{
		Task tsk, stsk;
		size_t t, st;
		for (t = 0; t < context.tasks.size(); t++)
		{
			auto it = m_mapStuff.sortedTasks.begin();
			tsk = context.tasks.at(t);
			//cout << "Sorting task(" << t << ") " << tsk.startTime << endl;
			for (st = 0; st < m_mapStuff.sortedTasks.size(); st++)
			{
				stsk = m_mapStuff.sortedTasks.at(st);
				//cout << "V task(" << st << ") " << stsk.startTime << endl;
				if (tsk.startTime < stsk.startTime)
				{
					//cout << "Less" << endl;
					break;
				}
			}
			if (0 == m_mapStuff.sortedTasks.size())
				m_mapStuff.sortedTasks.push_back(tsk);
			else
				m_mapStuff.sortedTasks.insert(it + st, tsk);
		}
	}

	Position PlannerHelper::aPointToPosition(aPoint pt, double altitude)
	{
		Position pos;
		pos.__set_longitude(pt.x());
		pos.__set_latitude(pt.y());
		pos.__set_altitude(altitude);
		return pos;
	}

	aPoint PlannerHelper::positionToAPoint(Position pos)
	{
		return aPoint(pos.longitude, pos.latitude);
	}

	void PlannerHelper::checkOnFirstTask(bool bProceed, Mission context, uint32_t vehicleIndex, pointVec &path)
	{
		aPoint point;

		/**
		If this is the first task, move from home position
		*/
		if (bProceed)
		{
			point = aPoint(
				context.homeLocation[vehicleIndex].longitude,
				context.homeLocation[vehicleIndex].latitude
			);
			path.push_back(point);
		}
	}

	void PlannerHelper::checkOnLastTask(bool bProceed, Mission &context, uint32_t vehicleIndex, pointVec &path)
	{
		aPoint point;

		/**
		If this is the last task, move to home position
		*/
		/*
		if (bProceed)
		{
			point = aPoint(
				context.homeLocation[vehicleIndex].longitude,
				context.homeLocation[vehicleIndex].latitude
			);
			path.push_back(point);
		}
		else
		*/
		point = path[path.size() - 1];

		double altitude = context.vehicles[vehicleIndex].stateVector.position.altitude;
		context.vehicles[vehicleIndex].stateVector.__set_position(aPointToPosition(point, altitude));
	}

	void PlannerHelper::transit(Mission &context, uint32_t taskIndex, bool bLastTask)
	{
		pointVec path;
		aPoint point;
		uint32_t vehicleIndex = findVehicleIndexById(context, m_mapStuff.sortedTasks[taskIndex].assignedVehicleId);

		preprocessingPathPlanning(context, taskIndex, vehicleIndex);

		checkOnFirstTask(0 == taskIndex, context, vehicleIndex, path);

		//-------------------------------------------------------------------
		if (taskIndex > 0)
		{
			if (context.tasks.at(taskIndex - 1).taskTemplate.taskType == TaskType().SURVEY)
			{
				m_mapStuff.start = positionToAPoint(context.vehicles[vehicleIndex].stateVector.position);
			}
		}
		//-------------------------------------------------------------------

		TaskAlgorithm ta;
		int32_t algo = 0;
		double resolution, resolutionDivider, heuristicWeight;

		if (getTaskAlgorithmDescr("TRANSIT", ta))
		{
			algo = getTaskAlgorithm(ta);
		}
		switch (algo)
		{
		default:
		case 0:
		{
			resolutionDivider = ta.parameters["Resolution Divider"];
			heuristicWeight = ta.parameters["Heuristic Weight"];

			if (0 == resolutionDivider)
				resolutionDivider = 50;
			if (0 == heuristicWeight)
				heuristicWeight = DEFAULT_ASTAR_HEURISTIC_WEIGHT;

			AStarPlanner::m_logNumber = taskIndex;
			resolution = hypot(m_mapStuff.navArea.width(), m_mapStuff.navArea.height()) / resolutionDivider;
			AStarPlanner planner(
				m_mapStuff.start, m_mapStuff.goal, m_mapStuff.obstacles_rect,
				m_mapStuff.navArea, resolution, heuristicWeight);

			planner.enableLog(ta.log);
			path = planner.plan();
			break;
		}
		case 3:
		{
			resolutionDivider = ta.parameters["Resolution Divider"];
			if (0 == resolutionDivider)
				resolutionDivider = 50;

			SPlanner::m_logNumber = taskIndex;
			double dx = m_mapStuff.sortedTasks[taskIndex].area.area[1].longitude - m_mapStuff.sortedTasks[taskIndex].area.area[0].longitude;
			double dy = m_mapStuff.sortedTasks[taskIndex].area.area[1].latitude - m_mapStuff.sortedTasks[taskIndex].area.area[0].latitude;
			resolution = hypot(dx, dy) / resolutionDivider;

			SPlanner planner(
				m_mapStuff.start, m_mapStuff.goal, m_mapStuff.obstacles_rect,
				m_mapStuff.navArea, resolution);

			planner.enableLog(ta.log);
			path = planner.plan();
			break;
		}
		}

		preprocessingPositions(context, taskIndex, vehicleIndex);

		checkOnLastTask(bLastTask, context, vehicleIndex, path);

		logPath(m_mapStuff.sortedTasks[taskIndex], path);
#ifdef _WINDOWS
		postPath(taskIndex, path);
#endif // _WINDOWS

		fillCommands(context, path, taskIndex, vehicleIndex, false);
	}

	void PlannerHelper::survey(Mission &context, uint32_t taskIndex, bool bLastTask)
	{
		pointVec path;
		uint32_t vehicleIndex = findVehicleIndexById(context, m_mapStuff.sortedTasks[taskIndex].assignedVehicleId);

		preprocessingPathPlanning(context, taskIndex, vehicleIndex);

		checkOnFirstTask(0 == taskIndex, context, vehicleIndex, path);

		aPoint startPoint = positionToAPoint(context.vehicles[vehicleIndex].stateVector.position);

		TaskAlgorithm ta, tca;
		int32_t algo = 0;
		int32_t connectionAlgo = 0;
		int32_t decompositionType = 0;

		if (getTaskAlgorithmDescr("SURVEY", ta))
		{
			algo = getTaskAlgorithm(ta);
			decompositionType = (int32_t)ta.parameters["Decomposition Type"];
		}
		switch (algo)
		{
		default:
		case 2:
		{
			if (getTaskAlgorithmDescr("SURVEY_TRANSIT", tca))
			{
				connectionAlgo = getTaskAlgorithm(tca);
			}

			double cameraSpanAngle = 90.0;
			double bigBase = haversine(m_mapStuff.start, m_mapStuff.goal);
			double height = context.vehicles[vehicleIndex].stateVector.position.altitude;
			double span = 2.0*height*tan(cameraSpanAngle*DEG2RAD/2.0);

			double resolution = (span / EARTH_MEAN_RADIUS) * RAD2DEG;
			double resolution2 = hypot(m_mapStuff.navArea.width(), m_mapStuff.navArea.height()) / 100;
			double resolution3 = m_mapStuff.obstacleMinimumSize / 2;

#ifdef _WINDOWS
			TRACE(_T("TASK AREA DIAGONAL (%f) m\n"), bigBase);
			TRACE(_T("VEHICLE ALTITUDE   (%f) m\n"), height);
			TRACE(_T("VEHICLE GROUND SPAN(%f) m\n"), span);
			TRACE(_T("RESOLUTION (SPAN)  (%f) m\n"), resolution);
			TRACE(_T("RESOLUTION (NAV)   (%f) m\n"), resolution2);
			TRACE(_T("RESOLUTION (OBS)   (%f) m\n"), resolution3);
#endif // _WINDOWS

			CoveragePlanner planner(m_mapStuff.start, m_mapStuff.goal, m_mapStuff.obstacles_rect, m_mapStuff.randArea, startPoint, resolution3);
			planner.setDecompositionType(decompositionType);

			double resolutionDivider, extraParam1 = 0.0;
			switch (connectionAlgo)
			{
			case 0:
				resolutionDivider = ta.parameters["Resolution Divider"];
				extraParam1 = ta.parameters["Heuristic Weight"];

				if (0 == resolutionDivider)
					resolutionDivider = 50;
				if (0 == extraParam1)
					extraParam1 = DEFAULT_ASTAR_HEURISTIC_WEIGHT;

				resolution = hypot(m_mapStuff.navArea.width(), m_mapStuff.navArea.height()) / resolutionDivider;
				break;
			case 3:
				resolutionDivider = ta.parameters["Resolution Divider"];
				if (0 == resolutionDivider)
					resolutionDivider = 50;
				double dx = m_mapStuff.sortedTasks[taskIndex].area.area[1].longitude - m_mapStuff.sortedTasks[taskIndex].area.area[0].longitude;
				double dy = m_mapStuff.sortedTasks[taskIndex].area.area[1].latitude - m_mapStuff.sortedTasks[taskIndex].area.area[0].latitude;
				resolution = hypot(dx, dy) / resolutionDivider;
				break;
			}

			planner.setConnectionAlgorithm(connectionAlgo, resolution, extraParam1);

			path = planner.plan();
			break;
		}
		}

		checkOnLastTask(bLastTask, context, vehicleIndex, path);

		logPath(m_mapStuff.sortedTasks[taskIndex], path);
#ifdef _WINDOWS
		postPath(taskIndex, path);
#endif // _WINDOWS

		fillCommands(context, path, taskIndex, vehicleIndex, false);
	}

	void PlannerHelper::inspect(Mission &context, uint32_t taskIndex, bool bLastTask)
	{
		uint32_t vehicleIndex = findVehicleIndexById(context, m_mapStuff.sortedTasks[taskIndex].assignedVehicleId);

		preprocessingPathPlanning(context, taskIndex, vehicleIndex);

		pointVec path;

		checkOnFirstTask(0 == taskIndex, context, vehicleIndex, path);

		path.push_back(m_mapStuff.start);
		path.push_back(m_mapStuff.goal);

		checkOnLastTask(bLastTask, context, vehicleIndex, path);

		logPath(m_mapStuff.sortedTasks[taskIndex], path);
#ifdef _WINDOWS
		postPath(taskIndex, path);
#endif // _WINDOWS

		fillCommands(context, path, taskIndex, vehicleIndex, false);
	}

	std::vector<aPoint> PlannerHelper::denormalizePath(std::vector<aPoint> npath)
	{
		aPoint pt, npt;
		std::vector<aPoint> path;
		for (uint32_t n = 0; n < npath.size(); n++)
		{
			npt = npath.at(n);
			pt = aPoint(
				denormalize(npt.x(), m_mapStuff.absMinLon, m_mapStuff.absMaxLon),
				denormalize(npt.y(), m_mapStuff.absMinLat, m_mapStuff.absMaxLat)
			);
			path.push_back(pt);
		}
		return path;
	}

	/**
	Sets field area, start and goal points in both GPS and normalized coordinates, suitable for planning algorithms.
	*/
	/**
	INSPECT: HOME
	*/
	/**
	SURVEY, TRANSIT: HOME->START POS->TARGET POS
	*/
	void PlannerHelper::preprocessingPathPlanning(Mission &context, uint32_t taskIndex, uint32_t vehicleIndex)
	{
		/*
		m_mapStuff.start = aPoint(
			context.vehicles[vehicleIndex].stateVector.position.longitude,
			context.vehicles[vehicleIndex].stateVector.position.latitude
		);
		*/
		m_mapStuff.start = aPoint(
			m_mapStuff.sortedTasks[taskIndex].area.area[0].longitude,
			m_mapStuff.sortedTasks[taskIndex].area.area[0].latitude
		);

#ifdef _WINDOWS
		//::PostMessage(m_pWnd->m_hWnd, WM_MLP_POSITION, WP_MLP_START_PT, (LPARAM)&m_mapStuff.start);
#endif // _WINDOWS

		m_mapStuff.startN = normalizePoint(m_mapStuff.start);

		if (m_mapStuff.sortedTasks[taskIndex].taskTemplate.taskType != TaskType().INSPECT)
			m_mapStuff.goal = aPoint(
				m_mapStuff.sortedTasks[taskIndex].area.area[1].longitude,
				m_mapStuff.sortedTasks[taskIndex].area.area[1].latitude
			);
		else
			m_mapStuff.goal = aPoint(
				m_mapStuff.sortedTasks[taskIndex].area.area[0].longitude,
				m_mapStuff.sortedTasks[taskIndex].area.area[0].latitude
			);

#ifdef _WINDOWS
		//::PostMessage(m_pWnd->m_hWnd, WM_MLP_POSITION, WP_MLP_GOAL_PT, (LPARAM)&m_mapStuff.goal);
#endif // _WINDOWS

		m_mapStuff.goalN = normalizePoint(m_mapStuff.goal);
	}

	void PlannerHelper::preprocessingPositions(Mission &context, uint32_t taskIndex, uint32_t vehicleIndex)
	{
		Position goalPoint = m_mapStuff.sortedTasks[taskIndex].area.area[1];

		if (m_mapStuff.sortedTasks[taskIndex].taskTemplate.taskType == TaskType().SURVEY)
		{
			for (size_t nc = 0; nc < m_mapStuff.sortedTasks[taskIndex].area.area.size(); nc++)
			{
				if (
					(m_mapStuff.sortedTasks[taskIndex].area.area[nc].longitude != context.vehicles[vehicleIndex].stateVector.position.longitude) &&
					(m_mapStuff.sortedTasks[taskIndex].area.area[nc].latitude != context.vehicles[vehicleIndex].stateVector.position.latitude)
					)
				{
					goalPoint = m_mapStuff.sortedTasks[taskIndex].area.area[nc];
					break;
				}
			}
		}

		//NO ! context.vehicles[vehicleIndex].stateVector.position = goalPoint;
	}

	/*D2.6 Semantic middleware pag. 78*/
	Command	PlannerHelper::fillCommand(CommandType::type type, Task relatedTask, double startTime, double endTime, std::vector<double> params)
	{
		Command cmd;

		cmd.__set_commandStatus(TaskCommandStatus::NotAssigned);
		cmd.__set_relatedTask(relatedTask);
		cmd.__set_commandType(type);
		cmd.__set_id(m_id++);
		cmd.__set_startTime((int64_t)startTime);
		cmd.__set_endTime((int64_t)endTime);
		cmd.__set_params(params);
		return cmd;
	}

	/*
	SURVEY TASK: starts with VIDEO_START_CAPTURE, then NAV_WAYPOINT(S) and last VIDEO_STOP_CAPTURE.
	TRANSIT: (for aerial vehicles UAV AUAV RUAV) starts with NAV_TAKEOFF, then NAV_WAYPOINT(S) and last NAV_LAND
	TRANSIT: (for other vehicles) only NAV_WAYPOINT(S)
	INSPECT: only CAMERA_IMAGE
	*/
	void PlannerHelper::fillCommands(Mission &context, std::vector<aPoint> path, uint32_t taskIndex, uint32_t vehicleIndex, bool bLastTask)
	{
		Task task = m_mapStuff.sortedTasks[taskIndex];

		m_startTime = (double)task.startTime;
		m_endTime = (double)task.endTime;

		m_speed = task.speed;
		if (m_speed == 0.0)
			m_speed = 10.0;

		switch (task.taskTemplate.taskType)
		{
		case TaskType().SURVEY:
			fillCommandsForSurvey(context, path, task, vehicleIndex, bLastTask);
			break;
		case TaskType().TRANSIT:
			if (isAFlyingVehicle(context.vehicles[vehicleIndex].type))
				fillCommandsForTransitFlyingVehicles(context, path, task, vehicleIndex, bLastTask);
			else
				fillCommandsForTransitOtherVehicles(context, path, task, vehicleIndex, bLastTask);
			break;
		case TaskType().INSPECT:
			fillCommandsForInspect(context, path, task, vehicleIndex, bLastTask);
			break;
		}
	}

	double	PlannerHelper::altitudeMagic(Task task)
	{
		double alt;
		if (task.altitude > 0)
			alt = task.altitude;
		else if (task.area.area[0].altitude > 0)
			alt = task.area.area[0].altitude;
		else if (task.area.area[1].altitude > 0)
			alt = task.area.area[1].altitude;
		else
			alt = 30;
		return alt;
	}

	void PlannerHelper::fillCommandsForTransitFlyingVehicles(Mission &context, std::vector<aPoint> path, Task &task, uint32_t vehicleIndex, bool bLastTask)
	{
		aPoint previousWaypoint;
		Command cmd;
		double v, vMax, vAllowed, alt, pathLength, tTOL, tPath, tWP, t0 = m_startTime, t1 = m_endTime;
		size_t w;

		//------------------	MAGIC
		alt = altitudeMagic(task);
		//---------------------------
		pathLength = alt;
		previousWaypoint = path[0];
		for (w = 1; w < path.size(); w++)
		{
			pathLength += haversine(previousWaypoint, path.at(w));
		}
		pathLength += alt;
		tPath = t1 - t0;
		tTOL = tPath * alt / pathLength;
		v = pathLength / tPath;
		vAllowed = m_speed;
		vMax = context.vehicles[vehicleIndex].maxSpeed;
		//---------------------------

		cmd = fillCommand(CommandType::NAV_TAKEOFF, task, t0, t0 + tTOL, { 0, 0, path.at(0).x(), path.at(0).y(), alt });
		m_cmdList.push_back(cmd);
		t0 = t0 + tTOL;

		previousWaypoint = path[0];
		for (w = 1; w < path.size(); w++)
		{
			tWP = tPath * haversine(previousWaypoint, path.at(w)) / pathLength;
			cmd = fillCommand(CommandType::NAV_WAYPOINT, task, t0, t0 + tWP, { 0, 0, 0, 0, path.at(w).x(), path.at(w).y(), alt });
			m_cmdList.push_back(cmd);
			t0 = t0 + tWP;

			previousWaypoint = path[w];
		}
		w--;

		cmd = fillCommand(CommandType::NAV_LAND, task, t0, t1, { 0, 0, 0, path.at(w).x(), path.at(w).y(), alt });
		m_cmdList.push_back(cmd);
	}

	void PlannerHelper::fillCommandsForSurvey(Mission &context, std::vector<aPoint> path, Task &task, uint32_t vehicleIndex, bool bLastTask)
	{
		aPoint previousWaypoint;
		Command cmd;
		double v, vMax, vAllowed, alt, pathLength, tSC = 1, tPath, tWP, t0 = m_startTime, t1 = m_endTime;
		size_t w;

		//------------------	MAGIC
		alt = altitudeMagic(task);
		//---------------------------
		pathLength = 0.0;
		previousWaypoint = path[0];
		for (w = 1; w < path.size(); w++)
		{
			pathLength += haversine(previousWaypoint, path.at(w));
		}
		tPath = t1 - t0 - tSC - tSC;
		v = pathLength / tPath;
		vAllowed = m_speed;
		vMax = context.vehicles[vehicleIndex].maxSpeed;
		//---------------------------

		cmd = fillCommand(CommandType::VIDEO_START_CAPTURE, task, t0, t0 + tSC, { 0, 0, 0, path.at(0).x(), path.at(0).y(), alt });
		m_cmdList.push_back(cmd);
		t0 = t0 + tSC;

		previousWaypoint = path[0];
		for (size_t w = 1; w < path.size() - 1; w++)
		{
			tWP = tPath * haversine(previousWaypoint, path.at(w)) / pathLength;
			cmd = fillCommand(CommandType::NAV_WAYPOINT, task, t0, t0 + tWP, { 0, 0, 0, 0, path.at(w).x(), path.at(w).y(), alt });
			m_cmdList.push_back(cmd);
			t0 = t0 + tWP;

			previousWaypoint = path[w];
		}

		cmd = fillCommand(CommandType::VIDEO_STOP_CAPTURE, task, t0, t1, { 0, 0, 0, path.at(0).x(), path.at(0).y(), alt });
		m_cmdList.push_back(cmd);
	}

	void PlannerHelper::fillCommandsForInspect(Mission &context, std::vector<aPoint> path, Task &task, uint32_t vehicleIndex, bool bLastTask)
	{
		Command cmd;
		double alt, t0 = m_startTime, t1 = m_endTime;

		//------------------	MAGIC
		alt = altitudeMagic(task);
		//---------------------------
		cmd = fillCommand(CommandType::CAMERA_IMAGE, task, t0, t1, { 0, 0, 0, 0, path.at(0).x(), path.at(0).y(), alt });
		m_cmdList.push_back(cmd);
	}

	void PlannerHelper::fillCommandsForTransitOtherVehicles(Mission &context, std::vector<aPoint> path, Task &task, uint32_t vehicleIndex, bool bLastTask)
	{
		aPoint previousWaypoint;
		Command cmd;
		double dur, t = m_startTime;

		previousWaypoint = path[0];
		for (size_t w = 1; w < path.size(); w++)
		{
			dur = haversine(previousWaypoint, path.at(w));
			cmd = fillCommand(CommandType::NAV_WAYPOINT, task, t, t + dur, { 0, 0, 0, 0, path.at(w).x(), path.at(w).y(), task.altitude });
			m_cmdList.push_back(cmd);
			t = t + dur;

			previousWaypoint = path[w];
		}
	}

	void PlannerHelper::preparePlan(Mission &context)
	{
		loadAlgorithmConfiguration();

		preprocessingForOptimalPath(context);
		sortTasks(context);

#ifdef _WINDOWS
		postMissionInfo(context);
#endif // _WINDOWS

		for (size_t t = 0; t < m_mapStuff.sortedTasks.size(); t++)
		{
			/**	Check if current task is TRANSIT
			*/
			if (context.tasks.at(t).taskTemplate.taskType == TaskType().TRANSIT)
			{
				transit(context, t, t == m_mapStuff.sortedTasks.size() - 1);
			}
			/**	Check if current task is SURVEY
			*/
			else if (context.tasks.at(t).taskTemplate.taskType == TaskType().SURVEY)
			{
				survey(context, t, t == m_mapStuff.sortedTasks.size() - 1);
			}
			/**	Check if current task is INSPECT
			*/
			else if (context.tasks.at(t).taskTemplate.taskType == TaskType().INSPECT)
			{
				inspect(context, t, t == m_mapStuff.sortedTasks.size() - 1);
			}
		}

		context.__set_commands(m_cmdList);
		if (!checkPlan(context))
		{
#ifdef _WINDOWS
			::PostMessage(m_pWnd->m_hWnd, WM_MLP_INFO, WP_MLP_INFO_FAIL, (LPARAM)PlannerHelper::formatAndAllocateString(_T("Plan not feasible !")));
#else
			cout << "Plan not feasible !" << endl;
#endif // _WINDOWS
		}
	}

	bool PlannerHelper::checkPlan(Mission context)
	{
		int64_t t0, t1, tn = 0;
		for (int c = 0; c < context.commands.size(); c++)
		{
			t0 = context.commands[c].startTime;
			if (t0 != tn)
			{
				m_logfile << "Command " << c << ". " << "Start time(" << t0 << ") != Prev End time(" << tn << ")" << endl;
				return false;
			}
			t1 = context.commands[c].endTime;
			if (t0 > t1)
			{
				m_logfile << "Command " << c << ". " << "Start time(" << t0 << ") > End time(" << t1 << ")" << endl;
				return false;
			}
			tn = t1;
		}
		return true;
	}

#ifdef _WINDOWS
	void PlannerHelper::postRectangle(UINT msgId, WPARAM wParam, aRectangle aRect)
	{
		aRectangle *pr = new aRectangle(aRect);
		::PostMessage(m_pWnd->m_hWnd, msgId, wParam, (LPARAM)pr);
	}

	void PlannerHelper::postRegion(UINT msgId, WPARAM wParam, Region region)
	{
		Region *pr = new Region(region);
		::PostMessage(m_pWnd->m_hWnd, msgId, wParam, (LPARAM)pr);
	}

	void PlannerHelper::postPosition(UINT msgId, WPARAM wParam, Position position)
	{
		Position *pp = new Position(position);
		::PostMessage(m_pWnd->m_hWnd, msgId, wParam, (LPARAM)pp);
	}

	void PlannerHelper::postMissionInfo(Mission mission)
	{
		//postRectangle(WM_MLP_RAND_AREA, 0, m_mapStuff.randArea);
		//::PostMessage(m_pWnd->m_hWnd, WM_MLP_RAND_AREA, 0, (LPARAM)&m_mapStuff.randArea);

		postRegion(WM_MLP_NAV_AREA, 0, mission.navigationArea);
		//::PostMessage(m_pWnd->m_hWnd, WM_MLP_NAV_AREA, 0, (LPARAM)&mission.navigationArea);

		if (mission.forbiddenArea.size() > 0)
		{
			for (size_t na = 0; na < mission.forbiddenArea.size(); na++)
			{
				postRegion(WM_MLP_OBS_AREA, na, mission.forbiddenArea.at(na));
				//::PostMessage(m_pWnd->m_hWnd, WM_MLP_OBS_AREA, na, (LPARAM)&mission.forbiddenArea.at(na));
			}
		}
		for (uint32_t np = 0; np < mission.homeLocation.size(); np++)
		{
			postPosition(WM_MLP_HOME_POS, np, mission.homeLocation[np]);
			//::PostMessage(m_pWnd->m_hWnd, WM_MLP_HOME_POS, np, (LPARAM)&mission.homeLocation[np]);
		}
		for (uint32_t nv = 0; nv < mission.vehicles.size(); nv++)
		{
			//::PostMessage(m_pWnd->m_hWnd, WM_MLP_VEHICLE_POS, nv, (LPARAM)&mission.vehicles.at(nv).stateVector.position);
			postPosition(WM_MLP_VEHICLE_POS, nv, mission.vehicles.at(nv).stateVector.position);
		}
		for (uint32_t nt = 0; nt < mission.tasks.size(); nt++)
		{
			postRegion(WM_MLP_TASK_AREA, nt, mission.tasks.at(nt).area);
			//::PostMessage(m_pWnd->m_hWnd, WM_MLP_TASK_AREA, nt, (LPARAM)&mission.tasks.at(nt).area);
		}
	}

	void PlannerHelper::postPath(int taskIndex, std::vector<aPoint> path)
	{
		::PostMessage(m_pWnd->m_hWnd, WM_MLP_INFO, 0, (LPARAM)formatAndAllocateString(_T("Path computed ...")));
		UINT wParam;
		aPoint *p;

		TaskStruct *pTaskStruct = new TaskStruct;
		pTaskStruct->id = m_mapStuff.sortedTasks[taskIndex].id;
		pTaskStruct->taskType = m_mapStuff.sortedTasks[taskIndex].taskTemplate.taskType;

		::PostMessage(m_pWnd->m_hWnd, WM_MLP_PATH_BEGIN, taskIndex, (LPARAM)pTaskStruct);
		for (size_t w = 0; w < path.size(); w++)
		{
			p = new aPoint(path[w]);
			wParam = (taskIndex << 16) | w;
			TRACE(_T("AP(%f,%f)\n"), path[w].x(), path[w].y());
			::PostMessage(m_pWnd->m_hWnd, WM_MLP_WAYPOINT, wParam, (LPARAM)p);
		}
		::PostMessage(m_pWnd->m_hWnd, WM_MLP_PATH_END, taskIndex, 0);
	}
#endif // _WINDOWS

	void PlannerHelper::logPath(Task task, std::vector<aPoint> path)
	{
		string taskType = _TaskType_VALUES_TO_NAMES.at(task.taskTemplate.taskType);
		m_logfile << taskType <<"(" << task.id << ") PATH. " << path.size() << " WAYPOINTS" << endl;
		for (size_t p = 0; p < path.size(); p++)
			m_logfile << "(" << path[p].x() << "," << path[p].y() << ")";
		m_logfile << endl;
	}

}