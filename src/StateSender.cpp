#include <telerobotics/Constants.h>
#include <telerobotics/StateSender.h>

namespace dcauv {

void defaultStateTransmittedCallback(StateSender &stateSender) {}

static PacketBuilderPtr pb =
    CreateObject<DataLinkFramePacketBuilder>(DataLinkFrame::fcsType::crc16);

StateSender::StateSender() : _txService(this), _device(pb) {
  _maxPacketLength = MAX_PACKET_LENGTH;
  _txService.SetWork(&StateSender::_Work);
  _stateTransmittedCallback = &defaultStateTransmittedCallback;
  SetLogName("StateSender");

  _device.SetCommsDeviceId("operator");
  _fcsType = DataLinkFrame::crc16;

  _dlf = DataLinkFrame::BuildDataLinkFrame(_fcsType);
  auto dlfbuffer = _dlf->GetPayloadBuffer();
  _trp = TransportPDU::BuildTransportPDU();
  _trp->SetBuffer(dlfbuffer);
  _trp->SetSeqNum(0);
  _txStatePtr = _trp->GetPayloadBuffer();
  _stateSize = 0;

  SetLocalAddr(1);
  SetRemoteAddr(2);
}

void StateSender::Start() {
  _device.Start();
  _txService.Start();
}

void StateSender::SetState(int size, const void *data) {
  _stateMutex.lock();
  _stateSize = size;
  memcpy(_txStatePtr, data, _stateSize);
  _stateMutex.unlock();
}

void StateSender::_SendPacket() {
  if (_stateSize > 0) {
    if (!_device.BusyTransmitting()) {
      _stateMutex.lock();
      _dlf->PayloadUpdated(TransportPDU::OverheadSize + _stateSize);
      Log->debug("TX: sending packet with new state... (Seq: {}) (FS: {})",
                 _trp->GetSeqNum(), _dlf->GetFrameSize());
      _device << _dlf;
      _stateMutex.unlock();
      _trp->IncSeqNum();

      _device.WaitForDeviceReadyToTransmit();
    } else
      Log->warn("TX: device busy transmitting");
  } else {
    Log->warn("TX: desired state is not set yet");
    Utils::Sleep(1000);
  }
}

void StateSender::_Work() {
  Log->debug("TX: Waiting for device ready to transmit...");
  _device.WaitForDeviceReadyToTransmit();
  _SendPacket();
}

StateSender::~StateSender() {
  if (_txService.IsRunning())
    _txService.Stop();
  _device.Stop();
}

void StateSender::SetLocalAddr(int _addr) {
  _localAddr = _addr;
  if (_dlf) {
    _dlf->SetSrcDir(_localAddr);
  }
}

void StateSender::SetRemoteAddr(int _addr) {
  _remoteAddr = _addr;
  if (_dlf) {
    _dlf->SetDesDir(_remoteAddr);
  }
}

void StateSender::SetStateTransmittedCallback(f_notification _callback) {
  _stateTransmittedCallback = _callback;
}

void StateSender::SetLogLevel(cpplogging::LogLevel _level) {
  Loggable::SetLogLevel(_level);
  _device.SetLogLevel(_level);
}

void StateSender::SetLogName(string name) {
  Loggable::SetLogName(name);
  _device.SetLogName(name + ":CommsDeviceService");
}

void StateSender::LogToConsole(bool c) {
  Loggable::LogToConsole(c);
  _device.LogToConsole(c);
}

void StateSender::LogToFile(const string &filename) {
  Loggable::LogToFile(filename);
  _device.LogToFile(filename + "_service");
}

void StateSender::FlushLog() {
  Loggable::FlushLog();
  _device.FlushLog();
}

void StateSender::FlushLogOn(LogLevel level) {
  Loggable::FlushLogOn(level);
  _device.FlushLogOn(level);
}
}
