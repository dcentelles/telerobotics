#include <telerobotics/Constants.h>
#include <telerobotics/StateReceiver.h>

namespace dcauv {

void defaultStateReceivedCallback(StateReceiver &stateReceiver) {}

static PacketBuilderPtr pb =
    CreateObject<DataLinkFramePacketBuilder>(DataLinkFrame::fcsType::crc16);

StateReceiver::StateReceiver() : _rxService(this), _device(pb) {
  _maxPacketLength = MAX_PACKET_LENGTH;
  _rxService.SetWork(&StateReceiver::_Work);
  _stateReceivedCallback = &defaultStateReceivedCallback;
  SetLogName("StateReceiver");

  _device.SetCommsDeviceId("rov");
  _fcsType = DataLinkFrame::crc16;

  _dlf = DataLinkFrame::BuildDataLinkFrame(_fcsType);
  auto dlfbuffer = _dlf->GetPayloadBuffer();
  _trp = TransportPDU::BuildTransportPDU();
  _trp->SetBuffer(dlfbuffer);
  _rxStatePtr = _trp->GetPayloadBuffer();
  _stateSize = 0;

  SetLocalAddr(2);
  SetRemoteAddr(1);
}
void StateReceiver::Start() {
  _device.Start();
  _rxService.Start();
}

void StateReceiver::_Work() { // Wait for the next packet and call the callback
  Log->debug("RX: waiting for frames...");
  _device.WaitForFramesFromRxFifo();
  Log->debug("RX: new frames in FIFO ({} frames).", _device.GetRxFifoSize());
  bool packetsReceived = false;
  int fifoSize = 0;
  while ((fifoSize = _device.GetRxFifoSize()) > 0) {
    _stateMutex.lock();
    _device >> _dlf;
    _stateMutex.unlock();
    Log->debug("RX: packets in FIFO: {}", fifoSize);
    Log->debug("RX: received new packet with last state (Seq: {}) (FS: {})",
               _trp->GetSeqNum(), _dlf->GetFrameSize());
    packetsReceived = true;
  }
  if (packetsReceived) {
    Log->debug("RX: notify last received state...");
    _stateReceivedCallback(*this);
  }
}

int StateReceiver::GetState(void *data, int maxLength) {
  _stateMutex.lock();
  int dataSize = _dlf->GetPayloadSize() - _trp->OverheadSize;
  if (dataSize > maxLength) {
    Log->error("GetState: buffer size < state size. Copying first {} bytes",
               maxLength);
    dataSize = maxLength;
  }
  memcpy(data, _rxStatePtr, dataSize);
  _stateMutex.unlock();
}

StateReceiver::~StateReceiver() {
  if (_rxService.IsRunning())
    _rxService.Stop();
  _device.Stop();
}

void StateReceiver::SetLocalAddr(int _addr) {
  _localAddr = _addr;
  if (_dlf) {
    _dlf->SetSrcDir(_localAddr);
  }
}

void StateReceiver::SetRemoteAddr(int _addr) {
  _remoteAddr = _addr;
  if (_dlf) {
    _dlf->SetDesDir(_remoteAddr);
  }
}

void StateReceiver::SetStateReceivedCallback(f_notification _callback) {
  _stateReceivedCallback = _callback;
}

void StateReceiver::SetLogLevel(cpplogging::LogLevel _level) {
  Loggable::SetLogLevel(_level);
  _device.SetLogLevel(_level);
}

void StateReceiver::SetLogName(string name) {
  Loggable::SetLogName(name);
  _device.SetLogName(name + ":CommsDeviceService");
}

void StateReceiver::LogToConsole(bool c) {
  Loggable::LogToConsole(c);
  _device.LogToConsole(c);
}

void StateReceiver::LogToFile(const string &filename) {
  Loggable::LogToFile(filename);
  _device.LogToFile(filename + "_service");
}

void StateReceiver::FlushLog() {
  Loggable::FlushLog();
  _device.FlushLog();
}

void StateReceiver::FlushLogOn(LogLevel level) {
  Loggable::FlushLogOn(level);
  _device.FlushLogOn(level);
}
}
