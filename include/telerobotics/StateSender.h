#ifndef STATESENDER_H
#define STATESENDER_H

#include <cpplogging/cpplogging.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/DataLinkFrame.h>
#include <dccomms/TransportPDU.h>
#include <dccomms/Utils.h>
#include <mutex>

namespace dcauv {

using namespace dccomms;
using namespace cpplogging;

class StateSender : public Logger {
public:
  StateSender();
  ~StateSender();

  void SetState(int size, const void *data);

  void SetLocalAddr(int);
  void SetRemoteAddr(int);

  void Start();

  typedef std::function<void(StateSender &)> f_notification;
  void SetStateTransmittedCallback(f_notification);

  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void SetLogName(string name);
  virtual void FlushLog();
  virtual void FlushLogOn(LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const string &filename);

private:
  DataLinkFrame::fcsType _dlfcrctype;
  std::mutex _stateMutex;
  f_notification _stateTransmittedCallback;
  DataLinkFramePtr _dlf;
  TransportPDUPtr _trp;
  ServiceThread<StateSender> _txService;
  uint8_t _localAddr, _remoteAddr;
  CommsDeviceService _device;
  uint8_t *_txStatePtr;
  uint16_t _stateSize;
  DataLinkFrame::fcsType _fcsType;
  int _txRate;

  int _maxPacketLength;
  void _Work();
  void _SendPacket();
};
}
#endif // STATESENDER_H
