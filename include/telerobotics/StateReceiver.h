#ifndef STATERECEIVER_H
#define STATERECEIVER_H

#include <cpplogging/cpplogging.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/DataLinkFrame.h>
#include <dccomms/TransportPDU.h>
#include <dccomms/Utils.h>
#include <mutex>

namespace telerobotics {

using namespace dccomms;
using namespace cpplogging;

class StateReceiver : public Logger {
public:
  StateReceiver();
  ~StateReceiver();

  void SetLocalAddr(int);
  void SetRemoteAddr(int);

  void Start();

  typedef std::function<void(StateReceiver &)> f_notification;
  void SetStateReceivedCallback(f_notification);
  int GetState(void *data, int maxLength);

  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void SetLogName(string name);
  virtual void FlushLog();
  virtual void FlushLogOn(LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const string &filename);

private:
  DataLinkFrame::fcsType _dlfcrctype;
  std::mutex _stateMutex;
  f_notification _stateReceivedCallback;
  DataLinkFramePtr _dlf;
  TransportPDUPtr _trp;
  ServiceThread<StateReceiver> _rxService;
  uint8_t _localAddr, _remoteAddr;
  CommsDeviceService _device;
  uint8_t *_rxStatePtr;
  uint16_t _stateSize;
  DataLinkFrame::fcsType _fcsType;

  int _maxPacketLength;
  void _Work();
};
}

#endif // STATERECEIVER_H
