// This autogenerated skeleton file illustrates how to build a server.
// You should copy it to another filename to avoid overwriting it.

#include "MmtService.h"
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

class MmtServiceHandler : virtual public MmtServiceIf {
 public:
  MmtServiceHandler() {
    // Your initialization goes here
  }

  void stateVectorUpdate(const int32_t requestId, const StateVector& stateVector) {
    // Your implementation goes here
    printf("stateVectorUpdate\n");
  }

  void sensorDataUpdate(const int32_t requestId, const std::vector<SensorData> & sensorData) {
    // Your implementation goes here
    printf("sensorDataUpdate\n");
  }

  void sendPlan(const int32_t requestId, const Mission& plan) {
    // Your implementation goes here
    printf("sendPlan\n");
  }

  void sendError(const int32_t errorId, const std::string& errorMessage, const int32_t requestId) {
    // Your implementation goes here
    printf("sendError\n");
  }

  void sendMissionStatusReport(const int32_t missionId, const TaskCommandStatus::type status) {
    // Your implementation goes here
    printf("sendMissionStatusReport\n");
  }

  void sendTaskStatusReport(const int32_t missionId, const int32_t taskId, const TaskCommandStatus::type status) {
    // Your implementation goes here
    printf("sendTaskStatusReport\n");
  }

  void sendCommandStatusReport(const int32_t missionId, const int32_t commandId, const TaskCommandStatus::type status) {
    // Your implementation goes here
    printf("sendCommandStatusReport\n");
  }

  void sendAlarm(const int32_t missionId, const Alarm& alarm) {
    // Your implementation goes here
    printf("sendAlarm\n");
  }

  void ping(std::string& _return) {
    // Your implementation goes here
    printf("ping\n");
  }

};

int main(int argc, char **argv) {
  int port = 9090;
  shared_ptr<MmtServiceHandler> handler(new MmtServiceHandler());
  shared_ptr<TProcessor> processor(new MmtServiceProcessor(handler));
  shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
  shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
  shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

  TSimpleServer server(processor, serverTransport, transportFactory, protocolFactory);
  server.serve();
  return 0;
}

