#pragma once

#include "PlannerService.h"

#include <thrift/server/TSimpleServer.h>
#include <thrift/server/TThreadPoolServer.h>
#include <thrift/server/TThreadedServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TTransportUtils.h>
#include <thrift/TToString.h>

using namespace afarcloud;

class PlannerServiceHandler : virtual public PlannerServiceIf {
public:
	PlannerServiceHandler(
#ifdef _WINDOWS
		CWnd *pWnd
#endif // _WINDOWS
	);

	void computePlan(const int32_t requestId, const Mission& context);

	void ping(std::string& _return);

	bool sendPlanToMMt(const int32_t requestId, const Mission& context);
protected:
#ifdef _WINDOWS
	CWnd *m_pWnd;
#endif // _WINDOWS
};
