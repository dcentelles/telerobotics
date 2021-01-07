#include <cpplogging/Logger.h>
#include <dccomms/Packet.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <telerobotics/LogUtils.h>

namespace telerobotics {

using namespace dccomms;
using namespace cpplogging;
using namespace std;

char *buff_to_hex(size_t size, uint8_t *buf) {
  char *hexstr = (char *)malloc(size * 3 + 1);
  for (size_t i = 0; i < size; i++) {
    sprintf(hexstr + i * 3, "%02X ", buf[i]);
  }
  return hexstr;
}

char *buff_to_hex_no_gap(size_t size, uint8_t *buf) {
  char *hexstr = (char *)malloc(size * 2 + 1);
  for (size_t i = 0; i < size; i++) {
    sprintf(hexstr + i * 2, "%02X", buf[i]);
  }
  return hexstr;
}

void info_packet(shared_ptr<spdlog::logger> log, PacketPtr pkt) {
  auto msg = buff_to_hex(pkt->GetPacketSize(), pkt->GetBuffer());
  log->info("--- {}", msg);
  delete msg;
}
} // namespace telerobotics
