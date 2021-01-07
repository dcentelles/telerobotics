#ifndef TELEROBOTICS_LOGUTILS_HPP_
#define TELEROBOTICS_LOGUTILS_HPP_

#include <cpplogging/Logger.h>
#include <dccomms/Packet.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace telerobotics {

using namespace dccomms;
using namespace cpplogging;
using namespace std;

char *buff_to_hex(size_t size, uint8_t *buf);

char *buff_to_hex_no_gap(size_t size, uint8_t *buf);

void info_packet(shared_ptr<spdlog::logger> log, PacketPtr pkt);

} // namespace telerobotics

#endif
