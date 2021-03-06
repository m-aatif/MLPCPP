// This autogenerated skeleton file illustrates how to build a server.
// You should copy it to another filename to avoid overwriting it.

#include "MissionManagerService.h"
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using boost::shared_ptr;

using namespace  ::afarcloud;

class MissionManagerServiceHandler : virtual public MissionManagerServiceIf {
 public:
  MissionManagerServiceHandler() {
    // Your initialization goes here
  }

  void sendPlan(const int32_t requestId, const Mission& plan) {
    // Your implementation goes here
    printf("sendPlan\n");
  }

  void abortMissionPlan(std::string& _return, const int32_t missionId) {
    // Your implementation goes here
    printf("abortMissionPlan\n");
  }

  void abortVehiclePlan(std::string& _return, const int32_t vehicleId) {
    // Your implementation goes here
    printf("abortVehiclePlan\n");
  }

  void abortMissionPlanHard(std::string& _return, const int32_t missionId) {
    // Your implementation goes here
    printf("abortMissionPlanHard\n");
  }

  void abortVehiclePlanHard(std::string& _return, const int32_t vehicleId) {
    // Your implementation goes here
    printf("abortVehiclePlanHard\n");
  }

  void ping(std::string& _return) {
    // Your implementation goes here
    printf("ping\n");
  }

};

int main(int argc, char **argv) {
  int port = 9090;
  shared_ptr<MissionManagerServiceHandler> handler(new MissionManagerServiceHandler());
  shared_ptr<TProcessor> processor(new MissionManagerServiceProcessor(handler));
  shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
  shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
  shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

  TSimpleServer server(processor, serverTransport, transportFactory, protocolFactory);
  server.serve();
  return 0;
}

