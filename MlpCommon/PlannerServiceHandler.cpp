#ifdef _WINDOWS
#include "pch.h"
#endif // _WINDOWS

//#define	LOG_MISSION_AS_JSON

#include "PlannerServiceHandler.h"
#include "PlannerHelper.h"
#include "MmtService.h"
#include "NetConfig.h"
#include "Configurator.h"
#include <boost/chrono.hpp>

#include <iostream>
#include <stdexcept>
#include <sstream>

#include <thrift/transport/TSocket.h>

#ifdef _WINDOWS
#include "WinMessages.h"
#endif // _WINDOWS

using namespace std;
using namespace apache::thrift;
using namespace apache::thrift::concurrency;
using namespace apache::thrift::protocol;
using namespace apache::thrift::transport;
using namespace apache::thrift::server;

using namespace boost::chrono;

using namespace afarcloud;

PlannerServiceHandler::PlannerServiceHandler(
#ifdef _WINDOWS
	CWnd *pWnd
#endif // _WINDOWS
)
{
#ifdef _WINDOWS
	m_pWnd = pWnd;
#endif // _WINDOWS
}
#include <fstream>

void LogMission(std::string prefix, int32_t requestId, const Mission& context, PlannerHelper& helper)
{
	std::ostringstream oss1;
	oss1 << Configurator::getLogFolder() << "\\" << prefix << "Mission" << requestId << "_" << context.missionId << ".txt";
	std::string logFileName1 = oss1.str();
	std::ofstream logfile1(logFileName1);

	//helper.dumpMission(context, std::cout);
	helper.dumpMission(context, logfile1);

	logfile1.close();
}

void LogMissionJson(const int32_t requestId, const Mission& context, PlannerHelper& helper)
{
	std::ostringstream oss2;
	oss2 << Configurator::getLogFolder() << "\\Mission" << requestId << "_" << context.missionId << ".json";
	std::string logFileName2 = oss2.str();
	std::ofstream logfile2(logFileName2);

	helper.dumpMissionAsJson(context, logfile2);

	logfile2.close();
}

void PlannerServiceHandler::computePlan(const int32_t requestId, const Mission& context) {
	PlannerHelper plannerHelper
#ifdef _WINDOWS
	(m_pWnd)
#endif // _WINDOWS
	;

#ifdef _WINDOWS
	::PostMessage(m_pWnd->m_hWnd, WM_MLP_STATUS, WP_MLP_START, 0);
	::PostMessage(m_pWnd->m_hWnd, WM_MLP_INFO, 0, (LPARAM)PlannerHelper::formatAndAllocateString(_T("Computing plan ...")));
#else
	cout << "Computing Plan(" << requestId << ")" << endl;
#endif // _WINDOWS
	
	Mission mission = context;

	plannerHelper.changeMissionEndianness(mission);

	Configurator cfg("MLP.json");
	if (cfg.isMissionLogInEnabled())
	{
		LogMission("IN_", requestId, mission, plannerHelper);
	}

#ifdef LOG_MISSION_AS_JSON
	LogMissionJson(requestId, mission, plannerHelper);
#endif // LOG_MISSION_AS_JSON

	/**
		Planning
	*/
	steady_clock::time_point scStart = high_resolution_clock::now();
	plannerHelper.preparePlan(mission);
	steady_clock::time_point scStop = high_resolution_clock::now();
	duration<double> sec = scStop - scStart;
#ifdef _WINDOWS
	::PostMessage(m_pWnd->m_hWnd, WM_MLP_MISSION_DURATION, 0, (LPARAM)(int)(10000.0 * sec.count()));
#else
	cout << "Elapsed time : " << sec.count() << " seconds\n";
#endif // _WINDOWS

	if (cfg.isMissionLogOutEnabled())
	{
		LogMission("OUT_", requestId, mission, plannerHelper);
	}

	bool bOk = sendPlanToMMt(requestId, mission);
	if (bOk)
#ifdef _WINDOWS
		::PostMessage(m_pWnd->m_hWnd, WM_MLP_INFO, WP_MLP_INFO_SUCCESS, (LPARAM)PlannerHelper::formatAndAllocateString(_T("Done")));
#else
		cout << "Done" << endl;
#endif // _WINDOWS
	else
#ifdef _WINDOWS
		::PostMessage(m_pWnd->m_hWnd, WM_MLP_INFO, WP_MLP_INFO_FAIL, (LPARAM)PlannerHelper::formatAndAllocateString(_T("Error sending plan to MMT")));
#else
		cout << "Error sending plan to MMT" << endl;
#endif // _WINDOWS
}

void PlannerServiceHandler::ping(std::string& _return) {
	_return = "It's a me! MLP!";
}

bool PlannerServiceHandler::sendPlanToMMt(const int32_t requestId, const Mission& context)
{
	hostParamsMap hostMap;
	std::string hostName;
	int32_t hostPort;

	Configurator cfg("MLP.json");
	if (cfg.loadHostsConfiguration(hostMap))
	{
		try
		{
			hostName = hostMap["MMT"].name;
			hostPort = hostMap["MMT"].port;
		}
		catch (const std::exception&)
		{
			hostName = MMT_HOST_NAME;
			hostPort = MMT_HOST_PORT;
		}
#ifdef _WINDOWS
		wchar_t sH[128];
		PlannerHelper::s2ws(hostName, sH);
		::PostMessage(m_pWnd->m_hWnd, WM_MLP_INFO, 0, (LPARAM)PlannerHelper::formatAndAllocateString(_T("Sending plan to MMT [%s] [%d] ..."), sH, hostPort));
#else
		cout << "Sending plan to MMT [" << hostName << "][" << hostPort << "] ..." << endl;
#endif // _WINDOWS
	}

	boost::shared_ptr<TTransport> socket(new TSocket(hostName, hostPort));
	boost::shared_ptr<TTransport> transport(new TBufferedTransport(socket));
	boost::shared_ptr<TProtocol> protocol(new TBinaryProtocol(transport));
	MmtServiceClient client(protocol);

	try {
		std::string gpString;
		transport->open();
		client.ping(gpString);
		cout << "ping(" << gpString << ")" << endl;
		client.sendPlan(requestId, context);
		transport->close();
		return true;
	}
	catch (TException& tx) {
		cout << "ERROR: " << tx.what() << endl;
		return false;
	}
}
