#include "PlannerServiceHandler.h"
#include "../../Common/NetConfig.h"
#include <thrift/concurrency/ThreadManager.h>
#include <thrift/concurrency/PosixThreadFactory.h>
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/server/TThreadPoolServer.h>
#include <thrift/server/TThreadedServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TTransportUtils.h>
#include <thrift/TToString.h>

#include <iostream>
#include <stdexcept>
#include <sstream>

#include "Configurator.h"

using namespace std;
using namespace apache::thrift;
using namespace apache::thrift::concurrency;
using namespace apache::thrift::protocol;
using namespace apache::thrift::transport;
using namespace apache::thrift::server;

using namespace afarcloud;

int main(int argc, char **argv) {

	Configurator::checkConfigurationFolders();

	int port;

	hostParamsMap hostMap;
	Configurator cfg("./MLP.json");
	if (cfg.loadHostsConfiguration(hostMap))
	{
		try
		{
			port = hostMap["MLP"].port;
		}
		catch (const std::exception&)
		{
			port = MLP_HOST_PORT;
		}
	}
	else
	{
		cout << "Error: Unable to read configuration file" << endl;
	}

	WSADATA wsaData = {};
	WORD wVersionRequested = MAKEWORD(2, 2);
	int err = WSAStartup(wVersionRequested, &wsaData);
	boost::shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());
	boost::shared_ptr<PlannerServiceHandler> handler(new PlannerServiceHandler());
	boost::shared_ptr<TProcessor> processor(new PlannerServiceProcessor(handler));
	boost::shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
	boost::shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());

	TSimpleServer server(processor,
		serverTransport,
		transportFactory,
		protocolFactory);

	cout << "Starting the MLP server (port " << port << ") ..." << endl;
	server.serve();
	cout << "Done." << endl;
	return 0;
}

